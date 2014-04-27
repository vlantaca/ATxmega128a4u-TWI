/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA TWI driver example source.
 *
 *      This file contains an example application that demonstrates the TWI
 *      master and slave driver. It shows how to set up one TWI module as both
 *      master and slave, and communicate with itself.
 *
 *      The recommended test setup for this application is to connect 10K
 *      pull-up resistors on PC0 (SDA) and PC1 (SCL). Connect a 10-pin cable
 *      between the PORTD and SWITCHES, and PORTE and LEDS.
 *
 * \par Application note:
 *      AVR1308: Using the XMEGA TWI
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 2660 $
 * $Date: 2009-08-11 12:28:58 +0200 (ti, 11 aug 2009) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include <util/delay.h>
#include "TC_driver.h"

#define SLAVE_ADDRESS    0b00101000

#define CPU_SPEED   2000000  // Hz
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define AM_FREQUENCY         2000  // Hz
#define MANCHESTER_FREQUENCY 100   // Hz

#define TIMER_PERIOD CPU_SPEED/AM_FREQUENCY
#define PULSE_COUNT  AM_FREQUENCY/(2*MANCHESTER_FREQUENCY)

/* Global variables */
TWI_Master_t twiMaster;
/**/


void send_high ( ) {
	int c;
	TC_SetCompareA(&TCD0,TIMER_PERIOD/2);
	for (c=0; PULSE_COUNT > c ;++c) {
		TC_ClearOverflowFlag(&TCD0);
		while(TC_GetOverflowFlag(&TCD0) == 0);
	}
}


void send_low ( ) {
	int c;
	TC_SetCompareA(&TCD0,0);
	for (c=0; PULSE_COUNT > c ;++c) {
		TC_ClearOverflowFlag(&TCD0);
		while(TC_GetOverflowFlag(&TCD0) == 0);
	}
}


void send_byte ( const unsigned char bits ) {
	unsigned char mask = 0x01;
	int i;
	for (i=0; 8 > i ;++i) {
		if (bits & (mask << i)) send_high();
		else send_low();
	}
}


void send_high_manchester ( ) {
	send_low();
	send_high();
}


void send_low_manchester ( ) {
	send_high();
	send_low();
}


void send_byte_manchester ( const unsigned char bits ) {
	unsigned char mask = 0x01;
	int i;
	for (i=0; 8 > i ;++i) {
		if (bits & (mask << i)) send_high_manchester();
		else send_low_manchester();
	}
}


int main(void){
	
	
	/* Comment out the 3 lines below if you want to use
	   your own pull-up resistors (I recommend using 4.7k).
	*/
	PORTCFG.MPCMASK = 0x03;                                     // Configure several PINxCTRL registers.
	PORTC.PIN0CTRL =\
		(PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;   // Use the internal pull-up resistors on PORTC's TWI Ports.

	TWI_MasterInit(
		&twiMaster,
		&TWIC,
		TWI_MASTER_INTLVL_LO_gc,
		TWI_BAUDSETTING
	);

	PMIC.CTRL |= PMIC_LOLVLEN_bm;                               // Enable LO interrupt level.
	sei();                                                      // Enable interrupts.
	
	PORTD.DIR = 0x01;                                           // Enable output on PortD0
	
	TC0_ConfigWGM(&TCD0,TC_WGMODE_SS_gc);                       // Configure single slope mode.
	TC0_EnableCCChannels(&TCD0,TC0_CCAEN_bm);                   // Enable compare channel A.
	TC0_ConfigClockSource(&TCD0,TC_CLKSEL_DIV1_gc);             // Start the timer by setting a clock source.
	
	TC_SetPeriod(&TCD0,TIMER_PERIOD);
	TC_SetCompareA(&TCD0,TIMER_PERIOD/2);
	
	twiMaster.readData[0] = 0x00;
	twiMaster.readData[1] = 0x00;
	twiMaster.readData[2] = 0x00;
	twiMaster.readData[3] = 0x00;

	int i, j, k;
/*
	// Clear input buffer to iPhone with
	for (i = 0; i < 10; i++) {
		send_low();		
	}
	*/
	// Start edge
	/*for (j = 0; j < 100; j++) {
		send_high();
	}
	
	//while (1) {
		// Data
		send_high_manchester();
		send_low_manchester();
		send_low_manchester();
		send_low_manchester();
		send_high_manchester();
	//}
	
	// Clear input buffer to iPhone with for end of transmission
	for (k = 0; k < 3; k++) {
		send_low();
	}
	
	//while (1) {
		/*TWI_MasterRead(
			&twiMaster,
			SLAVE_ADDRESS,
			TWIM_READ_BUFFER_SIZE
		);                                                      // Begin a TWI transaction.
		while (twiMaster.status != TWIM_STATUS_READY);          // Wait until the transaction completes.
													
		int i;
		for (i=TWIM_READ_BUFFER_SIZE-1; 0 <= i ;--i) {          // Iterate through the results.
			send_byte_manchester(twiMaster.readData[i]);        // Send the data using Manchester encoding.
		}
		*/
	//}
	send_high();
}


/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&twiMaster);
}
