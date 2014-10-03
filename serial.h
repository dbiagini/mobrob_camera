#ifndef F_CPU
#define F_CPU 8000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <util/delay.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (F_CPU/16/USART_BAUDRATE-1)

void out_char(char x);
char *convert(unsigned int num, int base);
char *convert_long(long num, int base);
unsigned char USART_Receive( void );
void USART_init(void);
void puts_UART(char* x);
void print_UART(char * frmt,...);

