CC = avr-gcc
CFLAGS = -Wall
LIBS = 
SRCS = $(wildcard *.c)
FUSES = -U lfuse:w:0x0e:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
compile: $(SRCS)
	@ ${CC} ${SRCS} -mmcu=atmega328 -o main.bin $(CFLAGS) $(LIBS)
	@ avr-objcopy -j .text -j .data -O ihex main.bin main.hex 
clean:
	@ rm -rf *.bin *.hex 
program:
	@ sudo avrdude -p m328 -c usbtiny -e -U flash:w:main.hex -v
