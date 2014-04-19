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

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0b00101000

/*! Defining number of bytes in buffer. */
//#define NUM_BYTES        4

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED   2000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */

/*! Buffer with test data to send.*/
//uint8_t sendBuffer[NUM_BYTES] = {0xCA, 0xFE, 0xBA, 0xBE}; // <--- for future communication with iPhone


int main(void){
	
	PORTA_DIR = 0xFF; // Set PORTA to be an output for LEDS

	// Use the internal pullup resistors on PORTC's TWI Ports
	// Comment out the two lines below if you want to use your own pullup resistors (I recommend using 4.7k)
	PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up 

	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	               &TWIC,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm; // PMIC controls the handling and prioritizing of interrupt requests
	sei(); // enables interrupts

	// Read 4 bytes of data from the I2C slave, (Chip Cap 2)
	TWI_MasterRead(&twiMaster, SLAVE_ADDRESS, 4);
	while (twiMaster.status != TWIM_STATUS_READY) {}; // Wait until transaction is complete.
			
	// Iterate through the bytes received, display the received data on the LEDS
	int i = 0;
	for (i=0; i!=4; i++){
		PORTA_OUT = twiMaster.readData[i];
		_delay_ms(500);
		PORTA_OUT = 0x00;
		_delay_ms(500);
	}


	
	/*********************************************************************************************
	* Do some initializations for using PWM
	*********************************************************************************************/
	uint16_t baseFrequency = 1352; // 1360 kHz (determined imperically)
	uint16_t desiredFrequency = 20; // how many Kilo Hertz dost thou desire?
	uint16_t periodValue = baseFrequency/desiredFrequency; // period value for PWM
	uint16_t compareValue = periodValue/2; // compare value for PWM
	
	// Enable output on PortD0
	PORTD.DIR = 0x01;
	
	
	/* Set the TC period. */
	//TC_SetPeriod( &TCD0, 0x0002 );
	//TC_SetCompareA( &TCD0, 0x0001 );
	
	periodValue = 101;
	TC_SetPeriod( &TCD0, periodValue );
	TC_SetCompareA( &TCD0, periodValue/2 );
	
	/* Configure the TC for single slope mode. */
	TC0_ConfigWGM( &TCD0, TC_WGMODE_SS_gc );

	/* Enable Compare channel A. */
	TC0_EnableCCChannels( &TCD0, TC0_CCAEN_bm );

	/* Start timer by selecting a clock source. */
	TC0_ConfigClockSource( &TCD0, TC_CLKSEL_DIV1_gc );
	int output_sel = 0;
	int counter = 0;
	
	
	// Main execution loop
	while(1){ 	
			
		/* Calculate new compare value. */
		//compareValue += 32;

		/* Output new compare value. */
		//TC_SetCompareA( &TCD0, 0x0001 );
		
		if (counter == 2500){ //this value determines the frequency of amplitude modulation

			if (output_sel == 1){ // turn output off (amplitude = 0)
				output_sel = 0;
				TC_SetCompareA( &TCD0, 0 );
			}else{ // turn output on (amplitude = Vcc)
				output_sel = 1;
				TC_SetCompareA( &TCD0, periodValue/2 );
			}
			counter = 0;
		}
		

		/************************************************
	    *  Wait for the new compare value to be latched
		*  from CCABUF[H:L] to CCA[H:L]. This happens at
		*  TC overflow (UPDATE ).
		************************************************/
		while( TC_GetOverflowFlag( &TCD0 ) == 0 ){};

		/* Clear overflow flag. */
		TC_ClearOverflowFlag( &TCD0 );
		counter++;

	} /* execution loop */
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&twiMaster);
}


