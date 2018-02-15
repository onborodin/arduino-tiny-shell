#
# $Id$
#

#.DEFAULT:
#.SUFFIXES: .c .o .hex .elf
#.PHONY:
.SECONDARY:

all: main.hex

CFLAGS= -I. -Os -DF_CPU=16000000UL -mmcu=atmega328p
#CFLAGS+= -fno-unwind-tables -fno-asynchronous-unwind-tables
#CFLAGS+= -ffunction-sections -fdata-sections
LDFLAGS= -s -O0 -DF_CPU=16000000UL -mmcu=atmega328p

main.elf: main.o fifo.o tools.o shell.o
	avr-gcc $(LDFLAGS) -o $@ main.o fifo.o tools.o shell.o
	avr-size --format=berkeley $@

roboarm.elf: roboarm.o 
	avr-gcc $(LDFLAGS) -o $@ roboarm.o 
	avr-size --format=berkeley $@


%.o: %.c
	avr-gcc $(CFLAGS) -c -o $@ $<

%.elf: %.o
	avr-gcc $(LDFLAGS) -o $@ $<

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@


%.upl: %.hex
	avrdude -qq -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 115200 -U flash:w:$<

upload: main.upl

backup:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 115200 -U flash:r:backup.hex:i


clean:
	rm -f *.i *.o *.elf *~ main.hex 

#EOF
