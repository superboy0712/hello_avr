/*****************************************************************
 * oled.h
 * hello_avr
 *
 *  Created on		: Sep 16, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/

#ifndef OLED_H_
#define OLED_H_
#include <inttypes.h>
#include <stdio.h>
void oled_init(void);
void oled_home(void);
void oled_goto_xy(uint8_t x,uint8_t y);
void oled_goto_nextln(void);
void oled_clear(void);
void oled_putchar( char c);
void oled_putstr( char * str);
void oled_wr_d(uint8_t data);
void oled_wr_cmd(uint8_t cmd);
void oled_set_normal(void);
void oled_set_inverse(void);
int oled_putchar_printf(char var, FILE *stream);
#endif /* OLED_H_ */
