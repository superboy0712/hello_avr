/*****************************************************************
 * lcm_12864.c
 * hello_avr
 *
 *  Created on		: Sep 16, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/
#include "lcm_12864.h"
#include "helpFile.h"
#include <util/delay.h>
//**********************************
//液晶初始化
//**********************************
void lcm_char_init(void)
{
	DDRC |= _BV(0)|_BV(1)|_BV(2);
	DDRA = 0Xff;
	lcm_wr_cmd(0x06);  /*光标的移动方向*/
	lcm_wr_cmd(0x0c);  /*开显示，关游标*/
	lcm_clr();
	lcm_char_gotoxy(1,1);
}
//***********************************
//填充液晶DDRAM全为空格
//**********************************
void lcm_clr (void)
{
	lcm_wr_cmd(0x30);
	lcm_wr_cmd(0x01);
}
//***********************************
//对液晶写数据
//content为要写入的数据
//***********************************
void lcm_wr_d(unsigned char content)
{
	busy_lcd();
	set_bit(RS_PORT,RS_BIT);
	clr_bit(RW_PORT,RW_BIT);
	lcddata=content;
	set_bit(E_PORT,E_BIT);
	_delay_us(10);
	clr_bit(E_PORT,E_BIT);
}
//********************************
//对液晶写指令
//content为要写入的指令代码
//*****************************
void lcm_wr_cmd(unsigned char content)
{
	busy_lcd();
	clr_bit(RS_PORT,RS_BIT);
    clr_bit(RW_PORT,RW_BIT);
	lcddata=content;
	set_bit(E_PORT,E_BIT);
	_delay_us(10);
	clr_bit(E_PORT,E_BIT);
}
//********************************
//液晶检测忙状态
//在写入之前必须执行
//********************************
void busy_lcd(void)
{
  lcddata=0xff;
  clr_bit(RS_PORT,RS_BIT);
  set_bit(RW_PORT,RW_BIT);
  set_bit(E_PORT,E_BIT);
  DDRA &= ~_BV(busy_BIT);
  clr_bit(busy_PORT,busy_BIT);
  while(test_bit(busy_PORT,busy_BIT));
  DDRA |= _BV(busy_BIT);
  clr_bit(E_PORT,E_BIT);
}
//********************************
//指定要显示字符的坐标
//*******************************
void lcm_char_gotoxy(unsigned char y, unsigned char x)
{
	if(y==1)
		lcm_wr_cmd(0x80|x);
	if(y==2)
        lcm_wr_cmd(0x90|x);
	if(y==3)
		lcm_wr_cmd((0x80|x)+8);
	if(y==4)
        lcm_wr_cmd((0x90|x)+8);
}
//**********************************
//液晶显示字符串程序
//**********************************
int lcm12864_putchar_printf(char var, FILE *stream)
{
	// translate \n to \r for br@y++ terminal
	    //if (var == '\n') wr_d_lcd('\n');
	    lcm_wr_d(var);
	    return 0;
}
/**
 * graphic mode init function
 */
void lcm_gr_init(void){
	DDRC |= _BV(0)|_BV(1)|_BV(2);
	DDRA = 0Xff;
	lcm_wr_cmd(0x34);
	lcm_wr_cmd(0x36);
}
void lcm_gr_clr(void){
	int i,j;
	lcm_wr_cmd(0x34);
	lcm_wr_cmd(0x36);
	for(i = 0; i<64; i++){
		lcm_gr_goto_16bit_addr(0,i);
		for(j = 0; j<256; j++)
			lcm_wr_d(0);
	}
}
/**
 *  graphic mode positioning. each data is 16bit, 2 bytes.
 *  cols: 16 (256 dots in a row. the last 128 dots won't be displayed
 *  can be used as external ram for buffering )
 *  rows: 64
 */
void lcm_gr_goto_16bit_addr( uint8_t col, uint8_t row ){

	lcm_wr_cmd(0x34);
	lcm_wr_cmd(0x36);
	lcm_wr_cmd(0b10000000|row);
	lcm_wr_cmd(0b10000000|col);
}
void lcm_gr_wr_data( uint16_t * data, uint8_t length){
	lcm_wr_cmd(0x34);
	lcm_wr_cmd(0x36);
	while(length--){
		lcm_wr_d((uint8_t)(*data/0xff));
		lcm_wr_d((uint8_t)(*data%0xff));
		data++;
	}
}
/**
 *  dot address,
 *  col: 128
 *  row: 64
 */
void lcm_gr_draw_pixel( uint8_t col, uint8_t row){
	lcm_gr_goto_16bit_addr( col/16, row);
	uint16_t data = _BV((16-col%16));
	lcm_gr_wr_data(&data,1);
}
/**
 *  rows: 0 - 63
 */
void lcm_gr_set_vertical_scroll(uint8_t rows){
	lcm_wr_cmd(0b00000011);// scroll mode
	rows &= 0b00111111;// valid input
	lcm_wr_cmd(0b01000000|rows);
}



