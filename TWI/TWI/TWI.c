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

#define SLAVE_ADDRESS 0b00101000

#define CPU_SPEED 2000000  // Hz
#define BAUDRATE   100000  // baud

#define SEND_DC    0x00
#define SEND_AC_AM 0x01
#define SEND_AC_FM 0x02

#define FREQUENCY          1000  // Hz
#define FREQUENCY_AM      20000  // Hz
#define FREQUENCY_FM_HIGH 10000  // Hz
#define FREQUENCY_FM_LOW  FREQUENCY_FM_HIGH/2  // Hz

#define PULSES(frequency) frequency/(2*FREQUENCY)

#define ENCODE_MANCHESTER 1  // [0,1]

TWI_Master_t twiMaster;


void timer_pulse ( ) {
    TC_ClearOverflowFlag(&TCD0);                                                          // Ensure terminal count is not falsely indicated.
    while(TC_GetOverflowFlag(&TCD0) == 0);                                                // Wait for the timer to reach terminal count.
}


void send_bit_DC ( const unsigned int bit ) {
    unsigned int period = CPU_SPEED/FREQUENCY;
    TC_SetPeriod(&TCD0,period);                                                           // Set the timer period with respect to the desired AM frequency.
    if (bit) TC_SetCompareA(&TCD0,period);                                                // Implement DC by using full duty cycle.
    else     TC_SetCompareA(&TCD0,0);                                                     // Implement logic low as GND (no duty cycle).
    int i;
    for (i=0; PULSES(FREQUENCY) > i ;++i) timer_pulse();                                  // Count pulses in order to control the frequency correctly.
}


void send_bit_AC_AM ( const unsigned int bit ) {
    unsigned int period = CPU_SPEED/FREQUENCY_AM;
    TC_SetPeriod(&TCD0,period);                                                           // Set the timer period with respect to the desired AM frequency.
    if (bit) TC_SetCompareA(&TCD0,period/2);                                              // Implement AC by using half duty cycle.
    else     TC_SetCompareA(&TCD0,0);                                                     // Implement logic low as GND (no duty cycle).
    int i;
    for (i=0; PULSES(FREQUENCY_AM) > i ;++i) timer_pulse();                               // Count pulses in order to control the frequency correctly.
}


void send_bit_AC_FM ( const unsigned int bit ) {
    unsigned int frequency = (bit) ? FREQUENCY_FM_HIGH : FREQUENCY_FM_LOW;                // Determine whether to send a logic high or logic low.
    unsigned int period = CPU_SPEED/frequency;                                            // Calculate the the timer period with respect to the desired FM frequency.
    TC_SetPeriod(&TCD0,period);                                                           // Set the timer period.
    TC_SetCompareA(&TCD0,period/2);                                                       // Implement AC by using half duty cycle.
    int i;
    for (i=0; PULSES(frequency) > i ;++i) timer_pulse();                                  // Count pulses in order to control the frequency correctly.
}


void send_bit ( const unsigned int bit , const unsigned char flags ) {
    void (*send_bit_method)(unsigned int);
    switch (flags) {
        case 0x00: send_bit_method = send_bit_DC;    break;                               // Send a DC bit.
        case 0x01: send_bit_method = send_bit_AC_AM; break;                               // Send an AM bit.
        case 0x02: send_bit_method = send_bit_AC_FM; break;                               // Send an FM bit.
        case 0x03:                                                                        // Simultaneous AM/FM is overkill for transmitting a single signal.
        default:
            return; // Error
    }
    if (ENCODE_MANCHESTER) send_bit_method(!bit);                                         // Manchester encoding -> (0 == 10 && 1 == 01). The 1st bit is the negation of the 2nd.
    send_bit_method(bit);                                                                 // Send the bit.
}


void send_byte ( const unsigned char bits , const unsigned char flags ) {
    unsigned char mask = 0x01;
    int i;
    for (i=0; 8 > i ;++i) {
        unsigned int bit = bits & (mask << i);                                            // Use a bitmask to determine the value of each bit to be sent.
        send_bit(bit,flags);                                                              // Send the bit.
    }
}


int main (void) {
    
    
    /* Comment out the 3 lines below if you want to use
       your own pull-up resistors (I recommend using 4.7k).
    */
    PORTCFG.MPCMASK = 0x03;                                                               // Configure several PINxCTRL registers.
    PORTC.PIN0CTRL =\
        (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;                             // Use the internal pull-up resistors on PORTC's TWI Ports.

    TWI_MasterInit(
        &twiMaster,
        &TWIC,
        TWI_MASTER_INTLVL_LO_gc,
        TWI_BAUD(CPU_SPEED,BAUDRATE)
    );

    PMIC.CTRL |= PMIC_LOLVLEN_bm;                                                         // Enable LO interrupt level.
    sei();                                                                                // Enable interrupts.
    
    PORTD.DIR = 0x01;                                                                     // Enable output on PortD0
    
    TC0_ConfigWGM(&TCD0,TC_WGMODE_SS_gc);                                                 // Configure single slope mode.
    TC0_EnableCCChannels(&TCD0,TC0_CCAEN_bm);                                             // Enable compare channel A.
    TC0_ConfigClockSource(&TCD0,TC_CLKSEL_DIV1_gc);                                       // Start the timer by setting a clock source.
    
    while (1) {
        TWI_MasterRead(
            &twiMaster,
            SLAVE_ADDRESS,
            TWIM_READ_BUFFER_SIZE
        );                                                                                // Begin a TWI transaction.
        while (twiMaster.status != TWIM_STATUS_READY);                                    // Wait until the transaction completes.
        int i;
        for (i=TWIM_READ_BUFFER_SIZE-1; 0 <= i ;--i) {                                    // Iterate through the results.
            send_byte(twiMaster.readData[i],SEND_AC_FM);                                  // Send the data using FM with Manchester encoding.
        }
    }
}


/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
    TWI_MasterInterruptHandler(&twiMaster);
}
