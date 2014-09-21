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
void init_lcd(void)
{
	DDRC |= _BV(0)|_BV(1)|_BV(2);
	DDRA = 0Xff;
	wr_i_lcd(0x06);  /*光标的移动方向*/
	wr_i_lcd(0x0c);  /*开显示，关游标*/
	clrram_lcd();
	gotoxy(1,1);
}
//***********************************
//填充液晶DDRAM全为空格
//**********************************
void clrram_lcd (void)
{
	wr_i_lcd(0x30);
	wr_i_lcd(0x01);
}
//***********************************
//对液晶写数据
//content为要写入的数据
//***********************************
void wr_d_lcd(unsigned char content)
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
void wr_i_lcd(unsigned char content)
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
void gotoxy(unsigned char y, unsigned char x)
{
	if(y==1)
		wr_i_lcd(0x80|x);
	if(y==2)
        wr_i_lcd(0x90|x);
	if(y==3)
		wr_i_lcd((0x80|x)+8);
	if(y==4)
        wr_i_lcd((0x90|x)+8);
}
//**********************************
//液晶显示字符串程序
//**********************************
int lcm12864_putchar_printf(char var, FILE *stream)
{
	// translate \n to \r for br@y++ terminal
	    //if (var == '\n') wr_d_lcd('\n');
	    wr_d_lcd(var);
	    return 0;
}





