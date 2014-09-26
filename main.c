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
static FILE oled_stdout =  FDEV_SETUP_STREAM(lcm12864_putchar_printf, NULL, _FDEV_SETUP_WRITE);
int main(void){
	unsigned int temp,scroll=0,hor=0;
	//stdout = &usart_stdout;
	usart_init(MYUBRR);
	lcm_gr_init();
	lcm_gr_clr();
	oled_init();
	_delay_ms(100);
	oled_init();
	oled_clear();
	oled_putstr("hello this is a ridiculously long long text string! enjoy it ! only for testing the line character alignment. I love you!!!");
	lcm_wr_cmd(0b00110100);
	while(1){
		temp = usart_getchar();
		usart_putchar(temp);
		if(temp==0x1B) {
			lcm_gr_clr();// ESC clear
		} else if(temp == 'a'){
			lcm_gr_set_vertical_scroll(--scroll);
		} else if(temp == 'z'){
			lcm_gr_set_vertical_scroll(++scroll);
		} else if(temp == 'g')
		{
			lcm_gr_clr();
			lcm_gr_goto_16bit_addr(0,++hor);
			lcm_wr_d(0xaa);
			lcm_wr_d(0xff);

		}else if(temp == 'h'){
			lcm_gr_clr();
						lcm_gr_goto_16bit_addr(0,--hor);
						lcm_wr_d(0xaa);
						lcm_wr_d(0xff);
		}

	}
	return 1;
}


