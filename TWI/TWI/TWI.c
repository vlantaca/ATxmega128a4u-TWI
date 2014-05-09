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
#include <math.h>
#include <util/delay.h>
#include "TC_driver.h"
#include "My_ADC.h"

#define SLAVE_ADDRESS    0b00101000

#define CPU_SPEED   2000000  // Hz
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define AM_FREQUENCY         15000  // Hz
#define MANCHESTER_FREQUENCY 25     // Hz

#define TIMER_PERIOD CPU_SPEED/AM_FREQUENCY
#define PULSE_COUNT  AM_FREQUENCY/(2*MANCHESTER_FREQUENCY)

/* Global variables */
TWI_Master_t twiMaster;
uint16_t adc_result;
uint8_t result_flag = 0;
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




ISR(ADCA_CH0_vect) {
	int test = 0;
	adc_result = ADCA.CH0.RES;
	
	//test = adc_result - 0x0550; //600-700mV threshold
	test = adc_result - 0x0384; ///400mv
	//test = adc_result - 0x0960; //1.2v
	
	if (test > 0x0000)
		result_flag = 1;
}




int main(void){
	/* Comment out the 3 lines below if you want to use
	   your own pull-up resistors (I recommend using 4.7k).
	*/
	PORTCFG.MPCMASK = 0x03;                                     // Configure several PINxCTRL registers.
	PORTC.PIN0CTRL =
		(PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;   // Use the internal pull-up resistors on PORTC's TWI Ports.

	TWI_MasterInit(
		&twiMaster,
		&TWIC,
		TWI_MASTER_INTLVL_LO_gc,
		TWI_BAUDSETTING
	);

	//PMIC.CTRL |= PMIC_LOLVLEN_bm;                               // Enable LO interrupt level.
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; //0x07
	sei();                                                      // Enable interrupts.

	PORTD.DIR = 0x01;                                           // Enable output on PortD0

	TC0_ConfigWGM(&TCD0,TC_WGMODE_SS_gc);                       // Configure single slope mode.
	TC0_EnableCCChannels(&TCD0,TC0_CCAEN_bm);                   // Enable compare channel A.
	TC0_ConfigClockSource(&TCD0,TC_CLKSEL_DIV1_gc);             // Start the timer by setting a clock source.

	TC_SetPeriod(&TCD0,TIMER_PERIOD);
	TC_SetCompareA(&TCD0,TIMER_PERIOD/2);
	
	/* **************************************
	 * DEBUG: Known packet values to confirm
	 *        communication is working.
	 *
	twiMaster.readData[0] = 0xDE;
	twiMaster.readData[1] = 0xAD;
	twiMaster.readData[2] = 0xBE;
	twiMaster.readData[3] = 0xEF;
	* ***************************************/
	//int rawHumidData[2];
	//int rawTempData[2];
	//float humidData = 0;
	//float tempData = 0;
	int i, j;
	int checkSum = 0;
	unsigned char mask = 0x01;

	// Initialize ADC
	adc_init();
	PORTB_DIR = 0xFF;
	PORTA_DIR = 0x00;
	
	send_low();
	
	while(1){
		// Start ADC conversion
		ADCA.CH0.CTRL |= 0x80;

		// Check if enable flag was set high
		if (result_flag){
			// Disable interrupts to send packet
			cli();
			
			/* **************************************
			 * DEBUG: writes a one to Port B0 when
			 *        the ADC value is past the high
			 *        threshold
			 */
			PORTB_OUT = 0x01;
			/***************************************/
			
			// Retrieve TWI communication to sensor
			TWI_MasterRead(
				&twiMaster,
				SLAVE_ADDRESS,
				TWIM_READ_BUFFER_SIZE);                    // Begin a TWI transaction.
			while (twiMaster.status != TWIM_STATUS_READY); // Wait until the transaction completes.
			
			// Check two most sig bits of twiMaster.readData[3] for input
			// status: 00B = Valid Data,            01B = Stale Data,
			//         10B = ChipCap2 Command Mode, 11B = Not Used
			/* Just sending raw data to iphone to be decoded
			if (!(twiMaster.readData[3] & 0xC0)) {
				// Grab humidity and temp data
				for (j=0; j < 6; j++) {
					rawHumidData[1] += twiMaster.readData[3] & (mask << j);
					rawTempData[0] += twiMaster.readData[0] & (mask << j+2);
				}
				rawHumidData[0] = twiMaster.readData[2];
				rawTempData[1] = twiMaster.readData[1];
			
				humidData = (rawHumidData[1]*256 + rawHumidData[0])/pow(2,14) * 100;
				tempData = (rawTempData[1]*64 + rawTempData[0]/4)/pow(2,14) * 165 -40;
			}
			*/
			// Reset and then create packet checksum
			checkSum = 0;
			for (i = 0; i < 4; i++) {
				for (j = 0; j < 8 ;j++) {
					if (twiMaster.readData[i] & (mask << j))
						checkSum += 1;
				}
			}
			
			// Clear input stream to iPhone then send start edge
			for (i = 0; i < 3; i++) {
				send_low();
			}
			send_high();
			
			// Send data
			for (i=TWIM_READ_BUFFER_SIZE-1; 0 <= i ;--i) {          // Iterate through the results.
				send_byte_manchester(twiMaster.readData[i]);      // Send the data using Manchester encoding.
			}
			
			// Send checksum
			send_byte_manchester(checkSum);
			
			// Clear input buffer to iPhone with
			for (i = 0; i < 3; i++) {
				send_low();
			}
			
			// Reset result_flag and Enable interrupts to check ADC
			result_flag = 0;
			sei();
		} else{
			/* **************************************
			 * DEBUG: writes a zero to Port B0 when
			 *        the ADC value is below the high
			 *        threshold
			 */
			PORTB_OUT = 0x00;
			/* *************************************/	
		}
	}
}


/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&twiMaster);
}