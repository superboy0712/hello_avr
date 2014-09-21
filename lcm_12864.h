/*****************************************************************
 * lcm_12864.h
 * hello_avr
 *
 *  Created on		: Sep 16, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/

#ifndef LCM_12864_H_
#define LCM_12864_H_
#include "pin_layout.h"
#include <stdio.h>
void wr_d_lcd(unsigned char content);
void wr_i_lcd(unsigned char content);
void clrram_lcd (void);
void init_lcd(void);
void busy_lcd(void);
void gotoxy(unsigned char y, unsigned char x);
int lcm12864_putchar_printf(char var, FILE *stream);
#endif /* LCM_12864_H_ */
