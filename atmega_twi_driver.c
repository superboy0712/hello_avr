/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  MEGA TWI master driver source file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the MEGA TWI master driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the MEGA TWI master module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * 
 * $Date: 2011-10-10 13:03:43 $  \n
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

#include "atmega_twi_driver.h"
#include <avr/io.h>
/*! \brief Maximum number of bytes to send including slave address byte and register address byte. */
#define NUM_BYTES 4
#define TWI_BUFFER_SIZE NUM_BYTES+1
/*
* Global variables
*/
static unsigned char TWI_buf[ TWI_BUFFER_SIZE ];    //!< Transceiver buffer
static unsigned char TWI_msgSize;                   //!< Number of bytes to be transmitted.
static unsigned char TWI_state = TWI_NO_STATE;      //!< State byte. Default set to TWI_NO_STATE.
union TWI_statusReg TWI_statusReg = {0};            //!< TWI_statusReg is defined in TWI_Master.h

/*! \brief Function to set up the TWI master to its initial standby state.
*/
void twi_master_initialise(void)
{
  TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
  TWSR = TWI_TWPS;                                  
  TWDR = 0xFF;                                      // Default content = SDA released.
	// ! internal pull up input state of SDA and SCL
	DDRSDA &= ~_BV(BITSDA);
	PORTSDA |= _BV(BITSDA);
  	
	DDRSCL &= ~_BV(BITSCL);
PORTSCL |= _BV(BITSCL);
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
         (0<<TWWC);                                 //
}    
    
/*! \brief Call this function to test if the TWI_ISR is busy transmitting.
*/
unsigned char twi_transceiver_busy( void )
{
  return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}

/*! \brief Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
* until the TWI_ISR has completed with the previous operation. If there was an error, then the function 
* will return the TWI State code. 
*/
unsigned char twi_get_state_info( void )
{
  while ( twi_transceiver_busy() );             // Wait until TWI has completed the transmission.
  return ( TWI_state );                         // Return error state.
}

/*! \brief Call this function to send a prepared message. The first byte must contain the slave address and the
* read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
* from the slave. Also include how many bytes that should be sent/read including the address byte.
* The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
* then initialize the next operation and return.
*/
void twi_start_transceiver_with_data( unsigned char *msg, unsigned char msgSize, unsigned char slave_addr)
{
  unsigned char temp;

  while ( twi_transceiver_busy() );             // Wait until TWI is ready for next transmission.

  TWI_msgSize = msgSize+2;                        // Number of data to transmit.
  TWI_buf[0]  = slave_addr;
  if (!(slave_addr & (TRUE<<TWI_READ_BIT) ))       // If it is a write operation, then also copy data.
  {
    if(msgSize)
	{
		for ( temp = 1; temp < msgSize+1; temp++ )
			TWI_buf[ temp ] = msg[ temp - 1 ];
	}
	
  }
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}

/*! \brief Call this function to resend the last message. The driver will reuse the data previously put in the transceiver buffers.
* The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
* then initialize the next operation and return.
*/
void twi_start_transceiver( void )
{
  while ( twi_transceiver_busy() );             // Wait until TWI is ready for next transmission.
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}


/*! \brief Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
* TWI_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
* data when they have arrived. Include a pointer to where to place the data and the number of bytes
* requested (including the address field) in the function call. The function will hold execution (loop)
* until the TWI_ISR has completed with the previous operation, before reading out the data and returning.
* If there was an error in the previous transmission the function will return the TWI error code.
*/
unsigned char twi_get_data_from_transceiver( unsigned char *msg, unsigned char msgSize )
{
  unsigned char i;

  while ( twi_transceiver_busy() );             // Wait until TWI is ready for next transmission.

  if( TWI_statusReg.lastTransOK )               // Last transmission competed successfully.              
  {                                             
    for ( i=0; i<msgSize; i++ )                 // Copy data from Transceiver buffer.
    {
      msg[ i ] = TWI_buf[ i ];
    }
  }
  return( TWI_statusReg.lastTransOK );                                   
}

/*! \brief This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
* that is whenever a TWI event has occurred. This function should not be called directly from the main
* application.
*/
ISR(TWI_vect)
{
  static unsigned char TWI_bufPtr;
  
  switch (TWSR)
  {
    case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      TWI_bufPtr = 0;                                     // Set buffer pointer to the TWI Address location
    case TWI_MTX_ADR_ACK:       // SLA+W has been tramsmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been tramsmitted and ACK received
      if (TWI_bufPtr < TWI_msgSize)
      {
        TWDR = TWI_buf[TWI_bufPtr++];
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
               (0<<TWWC);                                 //  
      }else                    // Send STOP after last byte
      {
        TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
               (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
               (0<<TWWC);                                 //
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK tramsmitted
      TWI_buf[TWI_bufPtr++] = TWDR;
    case TWI_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received
      if (TWI_bufPtr < (TWI_msgSize-1) )                  // Detect the last byte to NACK it.
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
               (0<<TWWC);                                 //  
      }else                    // Send NACK after next reception
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
               (0<<TWWC);                                 // 
      }    
      break; 
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK tramsmitted
      TWI_buf[TWI_bufPtr] = TWDR;
      TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
             (0<<TWWC);                                 //
      break;      
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received    
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
//    case TWI_NO_STATE              // No relevant state information available; TWINT = ??
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWI_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
                                                        // Reset TWI Interface
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
             (0<<TWWC);                                 //
  }
}
