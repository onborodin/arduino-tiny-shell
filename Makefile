#
# $Id$
#

#.DEFAULT:
#.SUFFIXES: .c .o .hex .elf
#.PHONY:
.SECONDARY:

all: main.hex

CFLAGS+= -I. -Os -DF_CPU=16000000UL -mmcu=atmega328p --std=c99
#CFLAGS+= -Wall
#CFLAGS+= -save-temps
#CFLAGS+= -fno-unwind-tables -fno-asynchronous-unwind-tables
CFLAGS+= -ffunction-sections -fdata-sections
CFLAGS+= -MD -MP -MT $(*F).o -MF $(@F).d 

LDFLAGS+= -s -DF_CPU=16000000UL -mmcu=atmega328p -lm
LDFLAGS+= -Wl,-u,vfprintf -lprintf_flt

main.elf: main.o fifo.o tools.o shell.o twim.o ds1307.o i2clcd.o md5.o adc.o mpu6050.o mpu6050dmp6.o
	avr-gcc $(LDFLAGS) -o $@ main.o fifo.o tools.o shell.o twim.o ds1307.o i2clcd.o adc.o md5.o mpu6050.o mpu6050dmp6.o
	avr-size --format=berkeley $@


twiscan.elf: twiscan.o fifo.o tools.o shell.o  twim.o ds1307.o i2clcd.o
	avr-gcc $(LDFLAGS) -o $@ twiscan.o fifo.o tools.o shell.o twim.o ds1307.o i2clcd.o
	avr-size --format=berkeley $@


lcd.elf: roboarm.o i2clcd.o
	avr-gcc $(LDFLAGS) -o $@ roboarm.o twim.o
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

%.upn: %.hex
	avrdude -qq -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 57600 -U flash:w:$<



upload: main.upl

backup:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 57600 -U flash:r:backup.hex:i


clean:
	rm -f *.i *.o *.s *.elf *~ *.hex *.d

#EOF
