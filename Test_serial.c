#define F_CPU 8000000L
#include"serial.h"

#define PI 3.14159265358979f
#define CT 125000.0f
int main (void)
{
  int i=0;
  int k=1250;
  char st[128]="string";
  

 USART_init();
  while(1){
  _delay_ms(330);
	out_char('T');
    _delay_ms(330); 
  i++;
  }
	return 0;

}
