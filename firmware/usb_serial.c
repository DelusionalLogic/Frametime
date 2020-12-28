#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usb_serial.h"

#ifdef NDEBUG
#define assert(x)
#else
#define assert(x) do{ if(!(x)) _assert(); }while(0)
#endif

static void _assert() {
	PORTD |= (1<<6);
	abort();
}

#define EP_TYPE_CONTROL           0x00
#define EP_TYPE_BULK_IN           0x81
#define EP_TYPE_BULK_OUT          0x80
#define EP_TYPE_INTERRUPT_IN      0xC1
#define EP_TYPE_INTERRUPT_OUT     0xC0
#define EP_TYPE_ISOCHRONOUS_IN    0x41
#define EP_TYPE_ISOCHRONOUS_OUT   0x40
#define EP_SINGLE_BUFFER          0x02
#define EP_DOUBLE_BUFFER          0x06
#define EP_SIZE(s) ((s) == 64 ? 0x30 : \
			((s) == 32 ? 0x20 : \
			((s) == 16 ? 0x10 : \
			             0x00)))

#define MAX_ENDPOINT 4

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#if defined(__AVR_AT90USB162__)
#define HW_CONFIG()
#define PLL_CONFIG()                 (PLLCSR = ((1<<PLLE)|(1<<PLLP0)))
#define USB_CONFIG()                 (USBCON = (1<<USBE))
#define USB_FREEZE()                 (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_ATmega32U4__)
#define HW_CONFIG()                  (UHWCON = 0x01)
#define PLL_CONFIG()                 (PLLCSR = 0x12)
#define USB_CONFIG()                 (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE()                 (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB646__)
#define HW_CONFIG()                  (UHWCON = 0x81)
#define PLL_CONFIG()                 (PLLCSR = 0x1A)
#define USB_CONFIG()                 (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE()                 (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB1286__)
#define HW_CONFIG()                  (UHWCON = 0x81)
#define PLL_CONFIG()                 (PLLCSR = 0x16)
#define USB_CONFIG()                 (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE()                 (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#endif

// standard control endpoint request types
#define GET_STATUS 0
#define CLEAR_FEATURE 1
#define SET_FEATURE 3
#define SET_ADDRESS 5
#define GET_DESCRIPTOR 6
#define GET_CONFIGURATION 8
#define SET_CONFIGURATION 9
#define GET_INTERFACE 10
#define SET_INTERFACE 11
// HID (human interface device)
#define HID_GET_REPORT 1
#define HID_GET_IDLE 2
#define HID_GET_PROTOCOL 3
#define HID_SET_REPORT 9
#define HID_SET_IDLE 10
#define HID_SET_PROTOCOL 11
// CDC (communication class device)
#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22

#include <stddef.h>

/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

// You can change these to give your code its own name.  On Windows,
// these are only used before an INF file (driver install) is loaded.
#define STR_MANUFACTURER	L"Delusional"
#define STR_PRODUCT		L"ScreenTimer"

// All USB serial devices are supposed to have a serial number
// (according to Microsoft).  On windows, a new COM port is created
// for every unique serial/vendor/product number combination.  If
// you program 2 identical boards with 2 different serial numbers
// and they are assigned COM7 and COM8, each will always get the
// same COM port number because Windows remembers serial numbers.
//
// On Mac OS-X, a device file is created automatically which
// incorperates the serial number, eg, /dev/cu-usbmodem12341
//
// Linux by default ignores the serial number, and creates device
// files named /dev/ttyACM0, /dev/ttyACM1... in the order connected.
// Udev rules (in /etc/udev/rules.d) can define persistent device
// names linked to this serial number, as well as permissions, owner
// and group settings.
#define STR_SERIAL_NUMBER	L"1"

// Mac OS-X and Linux automatically load the correct drivers.  On
// Windows, even though the driver is supplied by Microsoft, an
// INF file is needed to load the driver.  These numbers need to
// match the INF file.
#define VENDOR_ID		0x16C0
#define PRODUCT_ID		0x047A

// When you write data, it goes into a USB endpoint buffer, which
// is transmitted to the PC when it becomes full, or after a timeout
// with no more writes.  Even if you write in exactly packet-size
// increments, this timeout is used to send a "zero length packet"
// that tells the PC no more data is expected and it should pass
// any buffered data to the application that may be waiting.  If
// you want data sent immediately, call usb_serial_flush_output().
#define TRANSMIT_FLUSH_TIMEOUT	5   /* in milliseconds */

// If the PC is connected but not "listening", this is the length
// of time before usb_serial_getchar() returns with an error.  This
// is roughly equivilant to a real UART simply transmitting the
// bits on a wire where nobody is listening, except you get an error
// code which you can ignore for serial-like discard of data, or
// use to know your data wasn't sent.
#define TRANSMIT_TIMEOUT	25   /* in milliseconds */

// USB devices are supposed to implment a halt feature, which is
// rarely (if ever) used.  If you comment this line out, the halt
// code will be removed, saving 116 bytes of space (gcc 4.3.0).
// This is not strictly USB compliant, but works with all major
// operating systems.
#define SUPPORT_ENDPOINT_HALT



/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

// These buffer sizes are best for most applications, but perhaps if you
// want more buffering on some endpoint at the expense of others, this
// is where you can make such changes.  The AT90USB162 has only 176 bytes
// of DPRAM (USB buffers) and only endpoints 3 & 4 can double buffer.

#define ENDPOINT0_SIZE      16

#define KEYBOARD_ENDPOINT   1
#define CDC_ACM_ENDPOINT    2
#define CDC_RX_ENDPOINT     3
#define CDC_TX_ENDPOINT     4

#define KEYBOARD_SIZE       8
#define KEYBOARD_BUFFER     EP_SINGLE_BUFFER
#define CDC_ACM_SIZE        16
#define CDC_ACM_BUFFER      EP_SINGLE_BUFFER
#define CDC_RX_SIZE         16
#define CDC_RX_BUFFER       EP_DOUBLE_BUFFER
#define CDC_TX_SIZE         64
#define CDC_TX_BUFFER       EP_DOUBLE_BUFFER

#define ENDPOINT_CONFIG_LEN 4
struct Endpoint {
	uint8_t en;
	uint8_t type;
	uint8_t size;
};
static const struct Endpoint PROGMEM endpoint_config_table[ENDPOINT_CONFIG_LEN] = {
	{1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(KEYBOARD_SIZE) | KEYBOARD_BUFFER},
	{1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(CDC_ACM_SIZE) | CDC_ACM_BUFFER},
	{1, EP_TYPE_BULK_OUT,      EP_SIZE(CDC_RX_SIZE) | CDC_RX_BUFFER},
	{1, EP_TYPE_BULK_IN,       EP_SIZE(CDC_TX_SIZE) | CDC_TX_BUFFER}
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!

static const uint8_t PROGMEM device_descriptor[] = {
	18,                               // bLength
	1,                                // bDescriptorType
	0x00, 0x02,                       // bcdUSB
	0,                                // bDeviceClass
	0,                                // bDeviceSubClass
	0,                                // bDeviceProtocol
	ENDPOINT0_SIZE,                   // bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID),   // idVendor
	LSB(PRODUCT_ID), MSB(PRODUCT_ID), // idProduct
	0x00, 0x01,                       // bcdDevice
	1,                                // iManufacturer
	2,                                // iProduct
	3,                                // iSerialNumber
	1                                 // bNumConfigurations
};

#define DESC_CONFIG 0x02
#define DESC_INTERF 0x04
#define DESC_ENDPOI 0x05
#define DESC_CDC_INTERF 0x24

#define CONFIG_VALUE 0x01
#define CONFIG_SELFPOWERED 0xC0

#define INTERF_HID              0x03
#define INTERF_SUB_HID_BOOT     0x01
#define INTERF_PROTO_HID_KBD    0x01

#define INTERF_CDC_CLASS        0x02
#define INTERF_CDC_DATA         0x0A
#define INTERF_SUB_CDC_ACM      0x02
#define INTERF_PROTO_CDC_COM_AT 0x01


#define CDC_DESC_HEADER 0x00
#define CDC_DESC_CALL   0x01
#define CDC_DESC_ACM    0x02
#define CDC_DESC_UNION  0x06

#define CDC_CALL_HANDLES  0x01

#define CDC_ACM_SEND_BREAK    0x04
#define CDC_ACM_SERIAL_STATE  0x02

#define SERIAL_INTERFACE 1
#define KEYBOARD_INTERFACE 2

// Keyboard Protocol 1, HID 1.11 spec, Appendix B, page 59-60
static const uint8_t PROGMEM keyboard_hid_report_desc[] = {
        0x05, 0x01,          // Usage Page (Generic Desktop),
        0x09, 0x06,          // Usage (Keyboard),
        0xA1, 0x01,          // Collection (Application),
        0x75, 0x01,          //   Report Size (1),
        0x95, 0x08,          //   Report Count (8),
        0x05, 0x07,          //   Usage Page (Key Codes),
        0x19, 0xE0,          //   Usage Minimum (224),
        0x29, 0xE7,          //   Usage Maximum (231),
        0x15, 0x00,          //   Logical Minimum (0),
        0x25, 0x01,          //   Logical Maximum (1),
        0x81, 0x02,          //   Input (Data, Variable, Absolute), ;Modifier byte
        0x95, 0x01,          //   Report Count (1),
        0x75, 0x08,          //   Report Size (8),
        0x81, 0x03,          //   Input (Constant),                 ;Reserved byte
        0x95, 0x05,          //   Report Count (5),
        0x75, 0x01,          //   Report Size (1),
        0x05, 0x08,          //   Usage Page (LEDs),
        0x19, 0x01,          //   Usage Minimum (1),
        0x29, 0x05,          //   Usage Maximum (5),
        0x91, 0x02,          //   Output (Data, Variable, Absolute), ;LED report
        0x95, 0x01,          //   Report Count (1),
        0x75, 0x03,          //   Report Size (3),
        0x91, 0x03,          //   Output (Constant),                 ;LED report padding
        0x95, 0x06,          //   Report Count (6),
        0x75, 0x08,          //   Report Size (8),
        0x15, 0x00,          //   Logical Minimum (0),
        0x25, 0x68,          //   Logical Maximum(104),
        0x05, 0x07,          //   Usage Page (Key Codes),
        0x19, 0x00,          //   Usage Minimum (0),
        0x29, 0x68,          //   Usage Maximum (104),
        0x81, 0x00,          //   Input (Data, Array),
        0xc0                 // End Collection
};

#define CONFIG1_DESC_SIZE (9+9+5+5+4+5+7+9+7+7+9+9+7)
#define HID_DESC_OFFSET   (9+9+5+5+4+5+7+9+7+7+9)
static const uint8_t PROGMEM config1_descriptor[CONFIG1_DESC_SIZE] = {
	// configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
	9,                                         // bLength;
	DESC_CONFIG,                               // bDescriptorType;
	LSB(CONFIG1_DESC_SIZE),                    // wTotalLength
	MSB(CONFIG1_DESC_SIZE),
	3,                                         // bNumInterfaces
	CONFIG_VALUE,                              // bConfigurationValue
	0,                                         // iConfiguration
	CONFIG_SELFPOWERED,                        // bmAttributes
	50,                                        // bMaxPower
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,                                         // bLength
	DESC_INTERF,                               // bDescriptorType
	0,                                         // bInterfaceNumber
	0,                                         // bAlternateSetting
	1,                                         // bNumEndpoints
	INTERF_CDC_CLASS,                          // bInterfaceClass
	INTERF_SUB_CDC_ACM,                        // bInterfaceSubClass
	INTERF_PROTO_CDC_COM_AT,                   // bInterfaceProtocol
	0,                                         // iInterface
	// CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
	5,                                         // bFunctionLength
	DESC_CDC_INTERF,                           // bDescriptorType
	CDC_DESC_HEADER,                           // bDescriptorSubtype
	0x10, 0x01,                                // bcdCDC
	// Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
	5,                                         // bFunctionLength
	DESC_CDC_INTERF,                           // bDescriptorType
	CDC_DESC_CALL,                             // bDescriptorSubtype
	CDC_CALL_HANDLES,                          // bmCapabilities
	1,                                         // bDataInterface
	// Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
	4,                                         // bFunctionLength
	DESC_CDC_INTERF,                           // bDescriptorType
	CDC_DESC_ACM,                              // bDescriptorSubtype
	CDC_ACM_SEND_BREAK | CDC_ACM_SERIAL_STATE, // bmCapabilities
	// Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
	5,                                         // bFunctionLength
	DESC_CDC_INTERF,                           // bDescriptorType
	CDC_DESC_UNION,                            // bDescriptorSubtype
	0,                                         // bMasterInterface
	SERIAL_INTERFACE,                          // bSlaveInterface0
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                         // bLength
	DESC_ENDPOI,                               // bDescriptorType
	CDC_ACM_ENDPOINT | 0x80,                   // bEndpointAddress
	0x03,                                      // bmAttributes (0x03=intr)
	CDC_ACM_SIZE, 0,                           // wMaxPacketSize
	64,                                        // bInterval
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,                                         // bLength
	DESC_INTERF,                               // bDescriptorType
	SERIAL_INTERFACE,                          // bInterfaceNumber
	0,                                         // bAlternateSetting
	2,                                         // bNumEndpoints
	INTERF_CDC_DATA,                           // bInterfaceClass
	0x00,                                      // bInterfaceSubClass
	0x00,                                      // bInterfaceProtocol
	0,                                         // iInterface
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                         // bLength
	DESC_ENDPOI,                               // bDescriptorType
	CDC_RX_ENDPOINT,                           // bEndpointAddress
	0x02,                                      // bmAttributes (0x02=bulk)
	CDC_RX_SIZE, 0,                            // wMaxPacketSize
	0,                                         // bInterval
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                         // bLength
	DESC_ENDPOI,                               // bDescriptorType
	CDC_TX_ENDPOINT | 0x80,                    // bEndpointAddress
	0x02,                                      // bmAttributes (0x02=bulk)
	CDC_TX_SIZE, 0,                            // wMaxPacketSize
	0,                                         // bInterval
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,                                         // bLength
	DESC_INTERF,                               // bDescriptorType
	KEYBOARD_INTERFACE,                        // bInterfaceNumber
	0,                                         // bAlternateSetting
	1,                                         // bNumEndpoints
	INTERF_HID,                                // bInterfaceClass (0x03 = HID)
	INTERF_SUB_HID_BOOT,                       // bInterfaceSubClass (0x01 = Boot)
	INTERF_PROTO_HID_KBD,                      // bInterfaceProtocol (0x01 = Keyboard)
	0,                                         // iInterface
	// HID interface descriptor, HID 1.11 spec, section 6.2.1
	9,                                         // bLength
	0x21,                                      // bDescriptorType
	0x11, 0x01,                                // bcdHID
	0,                                         // bCountryCode
	1,                                         // bNumDescriptors
	0x22,                                      // bDescriptorType
	sizeof(keyboard_hid_report_desc),          // wDescriptorLength
	0,
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                         // bLength
	DESC_ENDPOI,                               // bDescriptorType
	KEYBOARD_ENDPOINT | 0x80,                  // bEndpointAddress
	0x03,                                      // bmAttributes (0x03=intr)
	KEYBOARD_SIZE, 0,                          // wMaxPacketSize
	1                                          // bInterval
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
	4,
	3,
	{0x0409} // Language ID
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
	sizeof(STR_MANUFACTURER),
	3,
	{STR_MANUFACTURER}
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
	sizeof(STR_PRODUCT),
	3,
	{STR_PRODUCT}
};
static const struct usb_string_descriptor_struct PROGMEM string3 = {
	sizeof(STR_SERIAL_NUMBER),
	3,
	{STR_SERIAL_NUMBER}
};

// This table defines which descriptor data is sent for each specific request
// from the host (in wValue and wIndex).
static struct descriptor {
	uint16_t wValue;
	uint16_t wIndex;
	const uint8_t *addr;
	uint8_t length;
} const PROGMEM descriptor_list[] = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x2200, KEYBOARD_INTERFACE, keyboard_hid_report_desc, sizeof(keyboard_hid_report_desc)},
	{0x2100, KEYBOARD_INTERFACE, config1_descriptor+HID_DESC_OFFSET, 9},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
	{0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor))


/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration=0;

// the time remaining before we transmit any partially full
// packet, or send a zero length packet.
static volatile uint8_t transmit_flush_timer=0;
static uint8_t transmit_previous_timeout=0;

#define KEYBOARD_KEYS_LEN 6
uint8_t keyboard_keys[KEYBOARD_KEYS_LEN] = {0, 0, 0, 0, 0, 0};
static uint8_t keyboard_protocol = 1;
static uint8_t keyboard_idle_config = 125;

// serial port settings (baud rate, control signals, etc) set
// by the PC.  These are ignored, but kept in RAM.
#define CDC_LINE_LEN 7
static uint8_t cdc_line_coding[CDC_LINE_LEN]={0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x08};
static uint8_t cdc_line_rtsdtr=0;

static void usb_wait_in_ready() {
	while (!(UEINTX & _BV(TXINI))) ;
}

static void usb_send_in() {
	UEINTX = ~_BV(TXINI);
}

static void usb_wait_receive_out() {
	while (!(UEINTX & _BV(RXOUTI))) ;
}

static void usb_ack_out() {
	UEINTX = ~_BV(RXOUTI);
}

static void setEP(uint8_t ep) {
	UENUM = ep;
}
static uint8_t getEP() {
	return UENUM;
}

static uint8_t rxBufferReadable(uint8_t intr) {
	return intr & _BV(RWAL);
}

static uint8_t txBufferWritable(uint8_t intr) {
	return intr & _BV(RWAL);
}

static uint8_t containsNewPacket(uint8_t intr) {
	return intr & _BV(RXOUTI);
}

static void txRelease() {
	UEINTX = 0x3A;
}

static void rxRelease() {
	UEINTX = 0x6B;
}

static uint8_t waitForInOrOut() {
	uint8_t intr;
	do {
		intr = UEINTX;
	} while (!(intr & (_BV(TXINI) | _BV(RXOUTI))));
	return intr & _BV(RXOUTI);
}

void usb_init() {
	HW_CONFIG();
	USB_FREEZE();                    // enable USB
	PLL_CONFIG();                    // config PLL, 16 MHz xtal
	while (!(PLLCSR & (1<<PLOCK))) ; // wait for PLL lock
	USB_CONFIG();                    // start USB clock
	UDCON = 0;                       // enable attach resistor
	usb_configuration = 0;
	cdc_line_rtsdtr = 0;
	UDIEN = (1<<EORSTE)|(1<<SOFE);
	sei();
}

uint8_t usb_configured() {
	return usb_configuration;
}

int16_t usb_serial_getchar() {
	if (!usb_configuration) return -1;

	uint8_t intr_state = SREG;
	cli();

	setEP(CDC_RX_ENDPOINT);
retry:;
	uint8_t  intr = UEINTX;
	if (!rxBufferReadable(intr)) {
		// no data in buffer
		if (containsNewPacket(intr)) {
			rxRelease();
			goto retry;
		}	
		SREG = intr_state;
		return -1;
	}
	// take one byte out of the buffer
	uint8_t c = UEDATX;
	// if buffer completely used, release it
	if (!rxBufferReadable(UEINTX))
		rxRelease();
	SREG = intr_state;
	return c;
}

// number of bytes available in the receive buffer
uint8_t usb_serial_available() {
	if(usb_configuration) return 0;

	uint8_t intr_state = SREG;
	cli();

	setEP(CDC_RX_ENDPOINT);
	uint8_t n = UEBCLX;
	if (!n) {
		uint8_t intr = UEINTX;
		if (containsNewPacket(intr) && !rxBufferReadable(intr))
			rxRelease();
	}

	SREG = intr_state;
	return n;
}

void usb_serial_flush_input() {
	if(!usb_configuration) return;

	uint8_t intr_state = SREG;
	cli();

	setEP(CDC_RX_ENDPOINT);
	while (rxBufferReadable(UEINTX)) {
		rxRelease();
	}

	SREG = intr_state;
}

// transmit a character.  0 returned on success, -1 on error
int8_t usb_serial_putchar(uint8_t c) {

	if (!usb_configuration) return -1;

	uint8_t intr_state = SREG;
	cli();
	setEP(CDC_TX_ENDPOINT);
	// if we gave up due to timeout before, don't wait again
	if (transmit_previous_timeout) {
		if (!txBufferWritable(UEINTX)) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	// wait for the FIFO to be ready to accept data
	uint8_t timeout = UDFNUML + TRANSMIT_TIMEOUT;
	while (1) {
		// are we ready to transmit?
		if (txBufferWritable(UEINTX)) break;
		SREG = intr_state;
		// have we waited too long?  This happens if the user
		// is not running an application that is listening
		if (UDFNUML == timeout) {
			transmit_previous_timeout = 1;
			return -1;
		}
		// has the USB gone offline?
		if (!usb_configuration) return -1;
		// get ready to try checking again
		intr_state = SREG;
		cli();
		setEP(CDC_TX_ENDPOINT);
	}
	// actually write the byte into the FIFO
	UEDATX = c;
	// if this completed a packet, transmit it now!
	if (!txBufferWritable(UEINTX))
			txRelease();
	transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	SREG = intr_state;
	return 0;
}

void serial_fast_select() {
	setEP(CDC_TX_ENDPOINT);
}

void serial_fast_write(uint8_t c) {
	assert(getEP() == CDC_TX_ENDPOINT);
	assert(txBufferWritable(UEINTX));

	// actually write the byte into the FIFO
	UEDATX = c;
}

void serial_fast_write16(uint16_t n) {
	assert(getEP() == CDC_TX_ENDPOINT);

	serial_fast_write(MSB(n));
	serial_fast_write(LSB(n));
}

void serial_fast_flush() {
	assert(getEP() == CDC_TX_ENDPOINT);

	txRelease();
}

// immediately transmit any buffered output.  This doesn't actually transmit
// the data - that is impossible!  USB devices only transmit when the host
// allows, so the best we can do is release the FIFO buffer for when the host
// wants it
void usb_serial_flush_output() {
	uint8_t intr_state;

	intr_state = SREG;
	cli();
	if (transmit_flush_timer) {
		setEP(CDC_TX_ENDPOINT);
		txRelease();
		transmit_flush_timer = 0;
	}
	SREG = intr_state;
}

uint8_t usb_serial_get_control() {
	return cdc_line_rtsdtr;
}


static void keyboard_write_report() {
	UEDATX = 0;
	UEDATX = 0;
	for (uint8_t i = 0; i < KEYBOARD_KEYS_LEN; i++) {
		UEDATX = keyboard_keys[i];
	}
}

int8_t usb_keyboard_press(uint8_t key) {
	keyboard_keys[0] = key;

	int8_t r = usb_keyboard_send();
	if (r) return r;

	keyboard_keys[0] = 0;
	return usb_keyboard_send();
}

int8_t usb_keyboard_ready() {
	if (!usb_configuration) return -1;

	setEP(KEYBOARD_ENDPOINT);
	uint8_t timeout = UDFNUML + 50;
	while (1) {
		// are we ready to transmit?
		if (txBufferWritable(UEINTX)) break;

		// has the USB gone offline?
		if (!usb_configuration) return -1;

		// have we waited too long?
		if (UDFNUML == timeout) return -1;

		// get ready to try checking again
		setEP(KEYBOARD_ENDPOINT);
	}

	return 0;
}

int8_t usb_keyboard_send() {
	if (!usb_configuration) return -1;

	uint8_t intr_state = SREG;
	cli();

	if(usb_keyboard_ready()) {
		SREG = intr_state;
		return -1;
	}

	keyboard_write_report();
	txRelease();

	SREG = intr_state;
	return 0;
}

int8_t usb_keyboard_send_sync() {

	if (!usb_configuration) return -1;

	uint8_t intr_state = SREG;
	cli();

	if(usb_keyboard_ready()) {
		SREG = intr_state;
		return -1;
	}

	keyboard_write_report();
	txRelease();

	// Wait for the next buffer to be available
	// This should ideally (since we are single buffered) mean the host has
	// ACKed our keyboard input, and should help reduce variance between return
	// from this function and the host polling
	// @CORRECTNESS: This does mean we return AFTER the host has received the
	// keypress, hopefully we are still fast enough that we can measure the
	// delay, but it could theoretically end up negative (and therefore
	// unmeasurable). A possible alternative could be to send an empty packet
	// at the start of this function, then immedietly schedule the keyboard
	// packet to MAXIMIZE the delay between we queue the keypress and the host
	// reads it. This makes it practically impossible to reach 0ms delay, but
	// would still synchronize us to the poll -Delusional 09/12-2020
	while (!(UEINTX & _BV(TXINI))) ;

	SREG = intr_state;
	return 0;
}

// USB Device Interrupt
ISR(USB_GEN_vect)
{

	uint8_t intbits = UDINT;
	UDINT = 0;
	if (intbits & _BV(EORSTI)) {
		setEP(0);
		UECONX = _BV(EPEN);
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = _BV(RXSTPE);
		usb_configuration = 0;
		cdc_line_rtsdtr = 0;
	}

	if ((intbits & _BV(SOFI)) && usb_configuration) {
		uint8_t t = transmit_flush_timer;
		if (t) {
			transmit_flush_timer = --t;
			if (!t) {
				setEP(CDC_TX_ENDPOINT);
				txRelease();
			}
		}
	}
}

struct USBReq {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

static void read_request(struct USBReq* req) {
	req->bmRequestType = UEDATX;
	req->bRequest = UEDATX;
	req->wValue = UEDATX | (UEDATX << 8);
	req->wIndex = UEDATX | (UEDATX << 8);
	req->wLength = UEDATX | (UEDATX << 8);
}

// USB Endpoint Interrupt
ISR(USB_COM_vect)
{
	setEP(0);
	uint8_t intbits = UEINTX;
	if (intbits & _BV(RXSTPI)) {
		const uint8_t *desc_addr;
		uint8_t	desc_length;

		struct USBReq req;
		read_request(&req);
		UEINTX = ~(_BV(RXSTPI) | _BV(RXOUTI) | _BV(TXINI));
		if (req.bRequest == GET_DESCRIPTOR) {
			for (uint8_t i = 0; i < NUM_DESC_LIST; i++) {
				const uint8_t* base = (const uint8_t*)(&descriptor_list[i]);
				uint16_t value = pgm_read_word(base + offsetof(struct descriptor, wValue));
				if (req.wValue != value) {
					continue;
				}
				uint16_t index = pgm_read_word(base + offsetof(struct descriptor, wIndex));
				if (req.wIndex != index) {
					continue;
				}
				desc_addr = (const uint8_t*)pgm_read_word(base + offsetof(struct descriptor, addr));
				desc_length = pgm_read_byte(base + offsetof(struct descriptor, length));
				goto success;
			}
			UECONX = _BV(STALLRQ) | _BV(EPEN);  //stall
			return;
success:;
			uint8_t len = (req.wLength < 255) ? req.wLength : 255;
			len = (desc_length < len) ? desc_length : len;
			do {
				if(waitForInOrOut()) return; // Abort if out packet is waiting

				// send IN packet
				uint8_t n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (uint8_t i = 0; i < n; i++) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len);
			return;
		}
		if (req.bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = req.wValue | _BV(ADDEN);
			return;
		}
		if (req.bRequest == SET_CONFIGURATION && req.bmRequestType == 0) {
			usb_configuration = req.wValue;
			cdc_line_rtsdtr = 0;
			transmit_flush_timer = 0;
			usb_send_in();
			for (uint8_t i = 0; i < ENDPOINT_CONFIG_LEN; i++) {
				const uint8_t* base = (uint8_t*)(&endpoint_config_table[i]);
				// Add one since EP 0 is global
				setEP(i+1);
				uint8_t en = pgm_read_byte(base + offsetof(struct Endpoint, en));
				UECONX = en;
				if (en) {
					UECFG0X = pgm_read_byte(base + offsetof(struct Endpoint, type));
					UECFG1X = pgm_read_byte(base + offsetof(struct Endpoint, size));
				}
			}
			UERST = 0x1E;
			UERST = 0;
			return;
		}
		if (req.bRequest == GET_CONFIGURATION && req.bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}
		if (req.bRequest == CDC_GET_LINE_CODING && req.bmRequestType == 0xA1) {
			usb_wait_in_ready();
			for (uint8_t i = 0; i < CDC_LINE_LEN; i++) {
				UEDATX = cdc_line_coding[i];
			}
			usb_send_in();
			return;
		}
		if (req.bRequest == CDC_SET_LINE_CODING && req.bmRequestType == 0x21) {
			usb_wait_receive_out();
			for (uint8_t i = 0; i < CDC_LINE_LEN; i++) {
				cdc_line_coding[i] = UEDATX;
			}
			usb_ack_out();
			usb_send_in();
			return;
		}
		if (req.bRequest == CDC_SET_CONTROL_LINE_STATE && req.bmRequestType == 0x21) {
			cdc_line_rtsdtr = req.wValue;
			usb_wait_in_ready();
			usb_send_in();
			return;
		}
		if (req.bRequest == GET_STATUS) {
			usb_wait_in_ready();
			uint8_t i = 0;
			if (req.bmRequestType == 0x82) {
				setEP(req.wIndex);
				if (UECONX & (1<<STALLRQ)) i = 1;
				setEP(0);
			}
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
		if ((req.bRequest == CLEAR_FEATURE || req.bRequest == SET_FEATURE)
				&& req.bmRequestType == 0x02 && req.wValue == 0) {
			uint8_t i = req.wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				setEP(i);
				if (req.bRequest == SET_FEATURE) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);
				} else {
					UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
					UERST = _BV(i);
					UERST = 0;
				}
				return;
			}
		}
		if (req.wIndex == KEYBOARD_INTERFACE) {
			if (req.bmRequestType == 0xA1) {
				if (req.bRequest == HID_GET_REPORT) {
					usb_wait_in_ready();
					keyboard_write_report();
					usb_send_in();
					return;
				}
				if (req.bRequest == HID_GET_IDLE) {
					usb_wait_in_ready();
					UEDATX = keyboard_idle_config;
					usb_send_in();
					return;
				}
				if (req.bRequest == HID_GET_PROTOCOL) {
					usb_wait_in_ready();
					UEDATX = keyboard_protocol;
					usb_send_in();
					return;
				}
			}
			if (req.bmRequestType == 0x21) {
				if (req.bRequest == HID_SET_REPORT) {
					usb_wait_receive_out();
					//keyboard_leds = UEDATX;
					uint8_t _ = UEDATX;
					usb_ack_out();
					usb_send_in();
					return;
				}
				if (req.bRequest == HID_SET_IDLE) {
					keyboard_idle_config = (req.wValue >> 8);
					usb_send_in();
					return;
				}
				if (req.bRequest == HID_SET_PROTOCOL) {
					keyboard_protocol = req.wValue;
					usb_send_in();
					return;
				}
			}
		}
	}
	UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}


