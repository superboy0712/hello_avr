/*****************************************************************
 * main.c
 * hello_avr
 *
 *  Created on		: Sep 9, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/

#define F_CPU 11059200UL

#include <util/delay.h>
#include <avr/io.h>
#include "helpFile.h"
#include <stdio.h>
#include "usart.h"
#include "lcm_12864.h"
#include "oled.h"
#include <avr/sfr_defs.h>
#include "temp2/atmega_twi_driver.h"
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0b1010000
static FILE usart_stdout =  FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

int main(void){
	stdout = &usart_stdout;
	usart_init(MYUBRR);

	uint8_t temp;
	//unsigned char receivedData [1] = {0x00};
	unsigned char success = 0x0;

	//Test
	//unsigned char Data[16];
	unsigned char receivedData [16];

	//! Initialize the driver
	atmel_led_drvr_init();
//	for(success = 0; success < 16;success++)
//	{
//		Data[success] = success+16;
//	}

	while(1){
		temp = usart_getchar();
		printf("input %c, ",temp);
		printf("status: %x \n\n", twi_get_state_info());

		if(temp=='1'){
			success = atmel_led_drvr_readarray(SLAVE_ADDRESS, 0, receivedData,8);
			if(!success){
				printf("readarray: error! no transaction! \r\n");
			} else{
				receivedData[8] = '\0';
				printf("readarray: %s \r\n", receivedData);
			}
		}else if('a' <= temp && temp <= 'z'){
			//! Write/read a register
			static uint8_t addr = 0;
			success = atmel_led_drvr_writeregister(SLAVE_ADDRESS, addr, temp);
			printf("writing %c to addr: %x \r\n", temp, addr);
			addr++;
		}
	}
	return 1;
}


