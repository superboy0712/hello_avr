/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  Atmel led driver example source.
 *
 *      This file contains an example application that demonstrates the master to drive a Atmel led slave driver. It shows how to initialize the driver and write
 *      and read the registers of the device.
 *
 *      The recommended test setup for this application is to connect 10K
 *      pull-up resistors (SDA) & (SCL) while using TWI
 *      Remember to connect 33 Ohm series resistor between MISO lines of TINYAVR and Atmel LED driver chip while using SPI interface
 *
 * \author
 *		gurbrinder.grewal
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1000 $
 * $Date: 2011-10-28 12:28:58 +0200  $  \n
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


/*! /brief Example code
 */
int twi_test(void)
{
	//! The variables have been defined as unsigned char to make the code compatible with ATXMEGA as well as ATMEGA compilers
	//! When using ATXMEGA, these variables should be defined as 'uint8_t' to avoid compiler warnings.
#define SLAVE_ADDRESS 0b1010000

	unsigned char REG_ADDR = 0x00;
	unsigned char REG_DATA = 0xAA;
	//unsigned char receivedData [1] = {0x00};
	unsigned char success = 0x0;
	
	//Test
	unsigned char Data[16];
	unsigned char receivedData [16];
	
	//! Initialize the driver
	atmel_led_drvr_init();
	for(success = 0; success < 16;success++)
	{
		Data[success] = success+16;		
	}
	
	//while (1)
	{
		
		//! Write/read a register
		success = atmel_led_drvr_writeregister(SLAVE_ADDRESS, REG_ADDR, REG_DATA);
		success = atmel_led_drvr_readregister(SLAVE_ADDRESS, REG_ADDR, &receivedData[0]);
		
		//! Write/read register array
		success = atmel_led_drvr_writearray(SLAVE_ADDRESS, REG_ADDR, Data, 16);
		success = atmel_led_drvr_readarray(SLAVE_ADDRESS, REG_ADDR, receivedData,16);
	}
	
	return 1;
}



