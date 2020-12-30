; vim: set comments+=\:;:
#include <avr/io.h>
__SP_H__ = 0x3E
__SP_L__ = 0x3D
__SREG__ = 0x3F
__tmp_reg__ = 0
__zero_reg__ = 1

CDC_TX_ENDPOINT = 4

.extern usb_keyboard_ready
.extern usb_keyboard_send
.extern keyboard_keys

; Branch of not zero
.macro brnz label
	brne \label
.endm

; This is 2 cycles
.macro serialwrite reg=r24
	sts _SFR_MEM_ADDR(UEDATX), \reg
.endm

; This is 3 cycles
.macro flush reg=r24
	ldi \reg, 0x3A
	sts _SFR_MEM_ADDR(UEINTX), \reg
.endm

.macro wait_for_buffer_ready reg=r24
.Wait\@:
	lds \reg, _SFR_MEM_ADDR(UEINTX)
	sbrs \reg, TXINI ; Escape the jump if bit is set
	rjmp .Wait\@
.endm

; The Sample loop is optimized to keep the ADC running at all times at 64
; prescaler. This means we "only' have 64 CPU cycles to start the ADC back up
; after a completed reading.
; The Sample loop is optimized keep the ADC 100% utilized at a prescaler value
; of 64. To do so the loop must take less than 64 cycles between ADSC being set
; low and us setting it high again.
; Additionally we want to keep the time stable, so the length of the loop has
; to match with the length of the inner WaitForADC loop to keep the total
; instruction stream length divisible by the ADC sample time. Since it takes
; the ADC 13 (ADC)cycles to compute the value, the total cycle length between
; successive ADSC low with 100% utilization is (13+1)*64=896.
.macro sample
.Sample\@:
	; Save the time
	lds r26, _SFR_MEM_ADDR(TCNT1L)
	lds r27, _SFR_MEM_ADDR(TCNT1H)
	; Reset the time
	sts _SFR_MEM_ADDR(TCNT1H), __zero_reg__
	sts _SFR_MEM_ADDR(TCNT1L), __zero_reg__ ; Sample_Length=29

	; Set ADSC bit to one to start ADC
	lds r16, _SFR_MEM_ADDR(ADCSRA)
	ori r16, _BV(ADSC)
	sts _SFR_MEM_ADDR(ADCSRA), r16 ; Sample_Length=34
	; @TIMING @ADCCLK: The sample happens exactly 1.5 ADC cycles after this.
	; Every ADC clock is 64 CPU cycles, meaning there's 96 cycles from here
	; till sample

	adiw r26, 8 ; Offset the difference between read and reset ; 35
	; @CORRECTNESS: I think i might be one cycle off in my math, maybe due to
	; the intra cycle timing?

	; High time
	serialwrite r27
	; Low time
	serialwrite r26 ; Sample_Length=40
	
	; We need nops here to align the WaitForADC loop to the ADC clock. The
	; number of nops is given by the formula
	; (896 - (WaitForADC_ExitLength + Sample_Length)) % .WaitForADC_LoopLength
	; Since:
	; - WaitForADC_ExitLength=4 (the cycles it takes from ADSC being set until
	; we exit the loop)
	; - Sample_Length=40 (The cycles from us exiting the loop until we reenter
	; WaitForADC excluding this padding), and
	; - WaitForADC_LoopLength=5 (The cycles it takes for one time around the
	; WaitForADC if ADSC is not set)
	; We need (896 - (40+4)) % 5 = 2 nops
	nop
	nop

	; If properly aligned, this loop should exit after 4 cycles.
.WaitForADC\@:
	lds r16, _SFR_MEM_ADDR(ADCSRA)
	andi r16, _BV(ADSC)
	brnz .WaitForADC\@

	; Sample_Length=0 <--- Counter starts here
	; Save the ADC value
	lds r17, _SFR_MEM_ADDR(ADCL)
	lds r16, _SFR_MEM_ADDR(ADCH)

	; Write the high ADC
	serialwrite r16

	; Write the low ADC
	serialwrite r17 ; Sample_Length=8

	; Flush every 16th record. r2 tracks how many samples we've taken. Keep in
	; mind that r2 might not be 0 on entry
	inc r2 ; Sample_Length=9
	; Flush if r2 is 16 (5th bit set)
	sbrs r2, 4
	rjmp .NoFlush\@
	clr r2
	flush r16
	rjmp .EndFlush\@
.NoFlush\@:
	; Some nops to take the same time as if we had flushed
	nop ; sbrs takes an additional cycle if it skips our jump
	
	nop ; clr

	nop ; flush
	nop
	nop
	; The rjmp is in either path, so ignore that
.EndFlush\@: ; Sample_Length=17

	; Count down
	sbiw r24, 1
	brnz .Sample\@ ; Sample_Length=21
.endm

; 16 cycles
.macro write_report reg
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), \reg
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
	sts _SFR_MEM_ADDR(UEDATX), __zero_reg__
.endm

.text
.global	doMeasure_inner
.type	doMeasure_inner, @function ; (uint8_t test_kc, uint8_t reset_kc)
doMeasure_inner:
	push r29
	push r28
	push r17
	push r16
	push r15
	push r2

	; Save kc for later
	mov r15, r24
	mov r28, r22

	; Enable timer with a 1/1 clock
	ldi r24, _BV(CS10)
	sts _SFR_MEM_ADDR(TCCR1B), r24

	; We dont want interrupts to screw with the timing
	cli

	; Make sure the usb serial is empty
	call usb_serial_flush_output

	; Switch to the keyboard interface and wait for the next buffer to be ready
	call usb_keyboard_ready

	; The first cycle of the ADC has a different timing from the rest. Just
	; cycle it once to even out the timing
	lds r24, _SFR_MEM_ADDR(ADCSRA)
	ori r24, _BV(ADEN) ; Enable the ADC
	ori r24, _BV(ADSC) ; Take a sample to init it
	sts _SFR_MEM_ADDR(ADCSRA), r24
.WaitForADC:
	lds r24, _SFR_MEM_ADDR(ADCSRA)
	sbrc r24, ADSC ; Escape the jump if bit is clear
	rjmp .WaitForADC
	
	; Send an empty report. To synchronize us to the usb host
	write_report __zero_reg__
	flush
	wait_for_buffer_ready

	; Assuming the pc polls us at 1000Hz we want to wait around a 16000 cycles
	; before we send the keypress to minimize the timing window. We still want
	; to leave a little air around it for timing differences though
	ldi r24, lo8(3890)
	ldi r25, hi8(3890)
.DelayLoop:
	sbiw r24, 1 ; 2 cycles
	brnz .DelayLoop ; 1 or 2 cycles

	; Reset the overflow
	in r25, _SFR_IO_ADDR(TIFR1)
	ori r25, _BV(TOV1)
	out _SFR_IO_ADDR(TIFR1), r25
	; Reset the time
	sts _SFR_MEM_ADDR(TCNT1H), __zero_reg__
	sts _SFR_MEM_ADDR(TCNT1L), __zero_reg__ ; Time starts here
	; This is the earliest possible time the computer could have recieved our
	; keypress

	; Send the keypress
	write_report r15
	flush
	; When the next buffer is ready the host must have read our data
	wait_for_buffer_ready

	; This is the latest possible time the host could have recieved our
	; keypress
	lds r17, _SFR_MEM_ADDR(TCNT1L) ; Time ends here
	lds r16, _SFR_MEM_ADDR(TCNT1H)
	; Reset the time
	sts _SFR_MEM_ADDR(TCNT1H), __zero_reg__
	sts _SFR_MEM_ADDR(TCNT1L), __zero_reg__

	; Select the serial usb interface again
	ldi r24, CDC_TX_ENDPOINT
	sts _SFR_MEM_ADDR(UENUM), r24

	serialwrite r16
	serialwrite r17

	; Reset the keyboard key. We don't really care how long this takes, so just
	; use the library function
	sts keyboard_keys, __zero_reg__
	call usb_keyboard_send ; selects the keyboard interface

	; select the serial usb interface again
	ldi r24, CDC_TX_ENDPOINT
	sts _SFR_MEM_ADDR(UENUM), r24

	ldi r24, 1 ; How many samples are already loaded
	mov r2, r24
	ldi r24, lo8(1024) ; How many new samples do we want
	ldi r25, hi8(1024)

	SAMPLE

	; Save if the overflow bit was set during the test because then the timing
	; could have been wrong
	in r16, _SFR_IO_ADDR(TIFR1)
	andi r16, _BV(TOV1)

	; Flush any remaining data
	call usb_serial_flush_output

	sei

	; Disable timer
	lds r24, _SFR_MEM_ADDR(TCCR1B)
	andi r24, ~_BV(CS10)
	andi r24, ~_BV(CS11)
	andi r24, ~_BV(CS12)
	sts _SFR_MEM_ADDR(TCCR1B), r24

	; Reset the keyboard key. We don't really care how long this takes, so just
	; use the library function
	sts keyboard_keys, r28
	call usb_keyboard_send ; selects the keyboard interface

	; Reset the keyboard key. We don't really care how long this takes, so just
	; use the library function
	sts keyboard_keys, __zero_reg__
	call usb_keyboard_send ; selects the keyboard interface

	; Return the overflow bit
	mov r24, r16
	ldi r25, 0

	pop r2
	pop r15
	pop r16
	pop r17
	pop r28
	pop r29
	ret	
