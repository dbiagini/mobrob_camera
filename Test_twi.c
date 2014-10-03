#define F_CPU 8000000L
#include"serial.h"
#include"twi_mini.h"
#include"MPU6050.h"
//#include"acc_plot.h"
#include<math.h>
#include"HMC5883l.h"
//#include"PWM.h"
//#include"matrix_math.h"

#define PI 3.14159265358979f
#define CT 125000.0f


static unsigned char read=1, write=0, address=0x68;

int main(void)
{
  unsigned char write_data=0x75, recv_data;
  int testn=0;
  USART_init();
  print_UART(" This is the test for TWI ");

  _delay_ms(1000);
  PORTD |= 0x80; ///turn led on
  _delay_ms(100);
  TWI_init_master();
  sei();
  print_UART(" Sending Start condition ");
  TWI_start();
  print_UART(" address= %x ", ((address<<1)|write));
  TWI_write_address((address<<1)|write);
  print_UART(" writing register = %x ", write_data);
  TWI_write_byte(write_data); //write register
  print_UART(" Repeat Start ");
  TWI_start(); //repeated start
  print_UART(" address read= %x ", ((address<<1)|read));
  TWI_write_address((address<<1)|read);
   print_UART(" read Data ");
  recv_data=TWI_read_byte(1); ///read byte and nack
  TWI_stop();
  print_UART(" This is the value of the register= %x  ", recv_data);
  PORTD &= 0x7F;
    
 
  return 0;
}

