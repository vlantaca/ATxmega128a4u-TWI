ATxmega128a4u-TWI
=================

Example of using the TWI interface of the ATxmega128a4u to communicate 
with a Chip Cap 2 TWI Device. The Atmel Studio project is TWI/TWI.atsln.
The hex file is located in TWI/TWI/Debug/TWI.hex. The hex file was
successfully loaded and tested on a MT-DB-X4 AVR Xmega development
board.

The main code is TWI/TWI/TWI.c, which currently reads a 4 bytes from
the Chip Cap 2 sensor, displays them on LEDS, and then outputs a
constant AM signal.
