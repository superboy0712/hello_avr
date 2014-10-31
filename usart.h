/*****************************************************************
 * usart.h
 * hello_avr
 *
 *  Created on		: Sep 16, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/

#ifndef USART_H_
#define USART_H_
#include <stdio.h>
#define USART_BAUD 19200
#define MYUBRR F_CPU/16/USART_BAUD-1
extern void usart_init(uint16_t ubrr);
extern char usart_getchar( void );
extern void usart_putchar( char data );
extern void usart_pstr(char *s);
int usart_putchar_printf(char var, FILE *stream);
#endif /* USART_H_ */
