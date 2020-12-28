#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <avr/cpufunc.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <iso646.h>

#include "usb_serial.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

uint8_t test_kc;
uint8_t reset_kc;

static void d_putchar(char c) {
	usb_serial_putchar(c);
}

static void d_putu16(uint16_t v) {
	usb_serial_putchar(MSB(v));
	usb_serial_putchar(LSB(v));
}

static void enableTimer() {
	// Enable Timer 1 with a 1/1 clock
	TCCR1B = _BV(CS10);
}

static void disableTimer() {
	// Disable Timer 1
	TCCR1B &= compl (_BV(CS10) | _BV(CS11) | _BV(CS12));
}

static void resetTimer() {
	TCNT1 = 0;
	TIFR1 |= _BV(TOV1);
}

static void pgm_send_str(const char *s) {
	char c;
	while (1) {
		c = pgm_read_byte(s++);
		if (!c) break;
		d_putchar(c);
	}
}

static uint8_t recv_str(char *buf, uint8_t size) {
	int16_t r;
	uint8_t count=0;

	while (count < size) {
		r = usb_serial_getchar();
		if (r != -1) {
			if (r == '\r' || r == '\n') return count;
			if (r >= ' ' && r <= '~') {
				*buf++ = r;
				count++;
			}
		} else {
			if (!usb_configured() || !(usb_serial_get_control() & USB_SERIAL_DTR)) {
				// user no longer connected
				return 255;
			}
			// just a normal timeout, keep waiting
		}
	}
	return count;
}

static inline uint8_t emitLevel(uint16_t level) {
	uint16_t time = TCNT1;
	uint8_t overflow = TIFR1 & _BV(TOV1);
	resetTimer();

	d_putu16(time);
	d_putu16(level);

	return overflow;
}

static uint8_t doCalibrate() {
	uint8_t err = 0;
	enableTimer();
	resetTimer();
	for(uint16_t i = 0; i < 100; i++) {
		// Start an ADC conversion by setting ADSC bit (bit 6)
		ADCSRA |= _BV(ADSC);
		// Wait until the ADSC bit has been cleared
		loop_until_bit_is_clear(ADCSRA, ADSC);
		uint16_t val = ADCL | (ADCH << 8);

		err |= emitLevel(val);
	}
	usb_serial_flush_output();

	disableTimer();
	return err;
}

extern uint8_t doMeasure_inner(uint8_t key);

static uint8_t doMeasure() {
	uint8_t err = 0;
	enableTimer();
	cli();

	usb_serial_flush_output();
	err = doMeasure_inner(test_kc);
	usb_serial_flush_output();

	sei();
	usb_keyboard_press(reset_kc);
	disableTimer();
	return err;
}

int main ()
{
	// Setup
	cli();

	// We want the full 16MHz
	CPU_PRESCALE(0);
	usb_init();

	// Enable the debug led
	LED_CONFIG;
	LED_OFF;

	MCUCR &= 0b11101111;
 
	//Set D4 as imput
	DDRD &= compl _BV(PIN4);

	// Configure the ADC
	// Use AVCC as reference, and select ADC8 as ADC input, full 10-bit precision, "high-speed" mode
	ADMUX = _BV(REFS0);
	ADCSRB |= _BV(MUX5) | _BV(ADHSM);
	// Select free running mode for the autotrigger
	ADCSRB &= compl (_BV(ADTS3) | _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0));
	// Set the prescaler to 64 (technically overclocking it)
	ADCSRA = _BV(ADPS2) | _BV(ADPS1);

	sei();

	// Actual Code

	while(!usb_configured()) ;

	char buf[32];
	while(1) {
		// Wait for the user to run their terminal emulator program which sets
		// DTR to indicate it is ready to receive.
		while (!(usb_serial_get_control() & USB_SERIAL_DTR)) ;

		// Discard anything that was received prior.  Sometimes the operating
		// system or other software will send a modem "AT command", which can
		// still be buffered.
		usb_serial_flush_input();

		pgm_send_str(PSTR("HELO ScreenTimer ready\n"));

		while(1) {
			uint8_t n = recv_str(buf, sizeof(buf));
			if (n == 255) break;

			if(n < 1) {
				pgm_send_str(PSTR("REJT\n"));
				continue;
			}

			if(buf[0] == 'C') {
				pgm_send_str(PSTR("CSTA\n"));
				if(doCalibrate()) {
					pgm_send_str(PSTR("\xFF\xFF\xFF\xFF"));
				} else {
					pgm_send_str(PSTR("\xFF\xFF\xFF\xFE"));
				}
			} else if(buf[0] == 'M') {
				pgm_send_str(PSTR("MSTA\n"));
				if(doMeasure()) {
					pgm_send_str(PSTR("\xFF\xFF\xFF\xFF"));
				} else {
					pgm_send_str(PSTR("\xFF\xFF\xFF\xFE"));
				}
			} else if(buf[0] == 'I') {
				// Write out the firmware configured CPU speed. It would be
				// better to get the ACTUAL CPU speed
				pgm_send_str(PSTR("RESL " TOSTRING(F_CPU) "\n"));
			} else if(buf[0] == 'K') {
				if(n < 3 || buf[1] != ' ' || n >= sizeof(buf)) {
					pgm_send_str(PSTR("REJT\n"));
					continue;
				}
				
				char* strend = buf + n;
				char* cursor = buf + 2;
				if(!isdigit(*cursor)) {
					pgm_send_str(PSTR("REJT\n"));
					continue;
				}

				char* firstarg = cursor;
				while(cursor != strend && isdigit(*cursor)) {
					cursor++;
				}

				if(cursor == strend || *cursor != ' ') {
					pgm_send_str(PSTR("REJT\n"));
					continue;
				}
				cursor++;

				if(cursor == strend || !isdigit(*cursor)) {
					pgm_send_str(PSTR("REJT\n"));
					continue;
				}

				char* secondarg = cursor;
				while(cursor != strend && isdigit(*cursor)) {
					cursor++;
				}
				*cursor = '\0';

				test_kc = atoi(firstarg);
				reset_kc = atoi(secondarg);

				pgm_send_str(PSTR("ACPT\n"));
			}
		}
	}

	return 1;
}
