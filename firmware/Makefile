PRG             = main
OBJ             = main.o usb_serial.o measure.o
MCU_TARGET      = atmega32u4
OPTIMIZE        = -O1
DEBUG           = -DNDEBUG
CPUFREQ         = 16000000UL
DUDECONF        = /usr/share/arduino/hardware/tools/avr/etc/avrdude.conf
PORT            = /dev/ttyACM0

DEFS           = -DF_CPU=$(CPUFREQ) -DBAUD=9600
LIBS           = -Ilibs -I.

# You should not have to change anything below here.

CC             = avr-gcc
AS             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(DEBUG) $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override ASFLAGS       = $(DEBUG) -mmcu=$(MCU_TARGET)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
main.o: main.c
example.o: example.c
usb_serial.o: usb_serial.c
measure.o: measure.S

%.flash: %.hex %.elf
	avrdude -D -p $(MCU_TARGET) -P $(PORT) -c arduino -b 115200 -V -F -U flash:w:$(@:.flash=.hex)
	avr-size --mcu=$(MCU_TARGET) -C $(@:.flash=.elf)

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf libs/*.o
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)
	rm -rf *.bin *.hex *.srec

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }
