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

#define USART_BAUD 19200
#define MYUBRR F_CPU/16/USART_BAUD-1

static FILE usart_stdout =  FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);
static FILE oled_stdout =  FDEV_SETUP_STREAM(oled_putchar_printf, NULL, _FDEV_SETUP_WRITE);
int main(void){
	int temp;
	//stdout = &usart_stdout;
	usart_init(MYUBRR);
	//init_lcd();
	oled_init();
	_delay_ms(100);
	oled_init();
	oled_clear();
	oled_putstr("hello this is a ridiculously long long text string! enjoy it ! only for testing the line character alignment. I love you!!!");

	while(1){
		temp = usart_getchar();
		usart_putchar(temp);
		if(temp==0x1B) {
			oled_clear();// ESC clear
		} else if(temp == 0x5c){
			oled_set_inverse();
		} else if(temp == 0x7c){
			oled_set_normal();
		} else
		{
			fprintf(&oled_stdout,"input: 0x%x, \'%c\'\n", temp,(char)temp);
		}

	}
	return 1;
}


