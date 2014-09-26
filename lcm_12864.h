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
void lcm_wr_d(unsigned char content);
void lcm_wr_cmd(unsigned char content);
void lcm_clr (void);
void lcm_char_init(void);
void busy_lcd(void);
void lcm_char_gotoxy(unsigned char y, unsigned char x);
void lcm_gr_init(void);
void lcm_gr_goto_16bit_addr( uint8_t col, uint8_t row );
void lcm_gr_wr_data( uint16_t * data, uint8_t length);
void lcm_gr_draw_pixel( uint8_t col, uint8_t row);
void lcm_gr_set_vertical_scroll(uint8_t rows);
int lcm12864_putchar_printf(char var, FILE *stream);
void lcm_gr_clr(void);
#endif /* LCM_12864_H_ */
