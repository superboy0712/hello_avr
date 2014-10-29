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
	_delay_us(1);
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
	_delay_us(1);
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
/**
 *  read data
 */
uint8_t lcm_rd_data(){
	uint8_t ret;

	busy_lcd();
	DDRA = 0x00; // input
	set_bit(RS_PORT,RS_BIT);
	set_bit(RW_PORT,RW_BIT);
	clr_bit(E_PORT,E_BIT);
	set_bit(E_PORT,E_BIT);
		_delay_us(1);
	ret = PINA;
	clr_bit(E_PORT,E_BIT);
	DDRA =0xff; // remember to set output again
	return ret;
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
	for(i = 0; i<32; i++){
		// the top and bottom half share the same 32x256 page
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
	col%=8;
	row%=64;
	if(row>=32){
		row -= 32;
		col += 8;
	}
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
	lcm_wr_cmd(0x34);
	lcm_wr_cmd(0x36);
	col%=128;
	row%=64;
	if(row>=32){
		row -= 32;
		col += 128;
	}
	lcm_wr_cmd(0b10000000|row);
	lcm_wr_cmd(0b10000000|(col/16));
	// dummy read
	lcm_rd_data();
	uint8_t high = lcm_rd_data();
	uint8_t low  = lcm_rd_data();
	lcm_wr_cmd(0b10000000|row);
	lcm_wr_cmd(0b10000000|(col/16));
	if(col%16 >= 8){
		lcm_wr_d(high);
		lcm_wr_d(low|(0x80>>(col%16-8)));
	} else {
		lcm_wr_d(high|(0x80>>(col%16)));
		lcm_wr_d(low);
	}
}
/**
 *  rows: 0 - 63
 */
void lcm_gr_set_vertical_scroll(uint8_t rows){
	lcm_wr_cmd(0b00000011);// scroll mode
	rows &= 0b00111111;// valid input
	lcm_wr_cmd(0b01000000|rows);
}
void glcd_set_pixel(uint8_t a, uint8_t b, uint8_t c){
	lcm_gr_draw_pixel(a,b);
}
void gr_draw_circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{

	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;


	glcd_set_pixel(x0, y0+r, color);
	glcd_set_pixel(x0, y0-r, color);
	glcd_set_pixel(x0+r, y0, color);
	glcd_set_pixel(x0-r, y0, color);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		glcd_set_pixel(x0 + x, y0 + y, color);
		glcd_set_pixel(x0 - x, y0 + y, color);
		glcd_set_pixel(x0 + x, y0 - y, color);
		glcd_set_pixel(x0 - x, y0 - y, color);

		glcd_set_pixel(x0 + y, y0 + x, color);
		glcd_set_pixel(x0 - y, y0 + x, color);
		glcd_set_pixel(x0 + y, y0 - x, color);
		glcd_set_pixel(x0 - y, y0 - x, color);

	}
}

