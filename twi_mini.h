#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define SLA_R_ACK 0x40
#define START_ACK 0x08
#define START_ACK_R 0x10
#define SLA_R_ACK_FAIL 0x38
#define SLA_R_NACK 0x48
#define DATA_R_ACK 0x50
#define DATA_R_NACK 0x58
#define SLA_W_ACK 0x18
#define SLA_W_NACK 0x20
#define DATA_W_ACK 0x28
#define DATA_W_NACK 0x30
#define SLA_W_DATA_FAIL 0x38

#define SCL_CLOCK  100000L

#define __DEBUG__

#include<avr/io.h>
#include<util/delay.h>
#ifdef __DEBUG__
#include"serial.h" ///for debugging purposes
#endif


void TWI_init_master(void);
unsigned char TWI_start(void);
unsigned char TWI_write_address(unsigned char);
unsigned char TWI_write_byte(unsigned char);
void TWI_stop(void);
unsigned char TWI_read_byte(unsigned char);
unsigned char TWI_read_bytes(unsigned char , unsigned char , unsigned char , unsigned char *);
unsigned char TWI_write_bytes(unsigned char, unsigned char, unsigned char, unsigned char *);
unsigned char TWI_write_bits(unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
unsigned char TWI_write_bit(unsigned char, unsigned char, unsigned char, unsigned char);///could be removed
unsigned char TWI_error(unsigned char);

