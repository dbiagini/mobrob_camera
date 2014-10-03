
#include "serial.h"

void out_char(char x){
	UDR=x;
	while(!(UCSRA & (1<<UDRE)));
return;
}

unsigned char USART_Receive( void )
{
  /* Wait for data to be received */
  while ( !(UCSRA & (1<<RXC)) )
  ;
  /* Get and return received data from buffer */
  return UDR;
}

//ISR(USART_RXC_vect){
//	char a;
//	a=UDR;
//	PORTB=a;
//}
void USART_init(void)
{
  UCSRB=(1<<RXEN)|(1<<TXEN); 
  UCSRC |= (1 << URSEL) | ( 1<< USBS)| (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
  UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
	sei();
  return;
}
void puts_UART(char* x){
  while(*x!='\0'){
    out_char(*x); 
    x++;  
  }
  return;  
}

char *convert(unsigned int num, int base)
{
  static char buff[33];
  char *ptr;
  ptr=&buff[sizeof(buff)-1];
  *ptr='\0';
  do
  {
    *--ptr="0123456789abcdef"[num%base];
    num/=base;
  }while(num!=0);
  return(ptr);
} 

char *convert_long(long num, int base)
{
  static char buff[33];
  char *ptr;
  ptr=&buff[sizeof(buff)-1];
  *ptr='\0';
  do
  {
    *--ptr="0123456789abcdef"[num%base];
    num/=base;
  }while(num!=0);
  return(ptr);
} 

int str_lenght(unsigned char* str){
int size=0;
while(*str!='\0'){
str++;
size++;
}
return size;
}

 void convert_digit_noTerm(int num, char* buff)
{
  if(buff){
    *buff="0123456789abcdef"[num%10];
  }   
  return;
} 

 void FloatToString(char *str, float f, char size){
        int pos,curr;  // position in string
        char len;  // length of decimal part of result
        char* str_beg;
        int value;  // decimal digit(s) to convert
        pos = 0;  // initialize pos, just to be sure
        curr = 0;
 
        value = (int)f;  // truncate the floating point number
        if (f < 0 )  // handle negative numbers
        {
                f *= -1;
                value *= -1;
                str[pos]='-';
                pos++;
        }
        str_beg=convert(value, 10);
        while(str_beg[curr]!='\0') {
          str[pos]=str_beg[curr]; ///copy integer
          pos++;
          curr++;
        }
        len=pos;
   
        str[pos++] = '.';  // add decimal point to string
        
        while((pos < (size-2))&&(f))  // process remaining digits
        {
                f = f - (float)value;  // hack off the whole part of the number
                f *= 10;  // move next digit over
                value = (int)f;  // get next digit
                convert_digit_noTerm(value, &str[pos++]);
        }
        str[pos++]='\0';  ///terminating character
 }

void print_UART(char * frmt,...)
{
  char *p;
  int i;
  unsigned u;
  char *s;
  float f;
  char float_out[33];
  va_list argp;
  va_start(argp, frmt);
  p=frmt;
  for(p=frmt; *p!='\0';p++)
  {
    if(*p!='%')
    {
      out_char(*p);
    }else{
      p++;
      switch(*p)
      {
        case 'c' : i=va_arg(argp,int);
                  out_char(i);
                  break;
        case 'd' : i= va_arg(argp,int);
          if(i<0){
            i=-i;
            out_char('-');
          } 
          puts_UART(convert(i,10));
          break;
        case 'o': i=va_arg(argp,unsigned int); puts_UART(convert(i,8));break;
        case 's': s=va_arg(argp,char *); puts_UART(s); break;
        case 'u': u=va_arg(argp, unsigned int); puts_UART(convert(u,10));break;
        case 'x': u=va_arg(argp, unsigned int); out_char('0'); out_char('x'); puts_UART(convert(u,16));break;
        case '%': out_char('%');break;
        case 'f': f=(float) va_arg(argp, double);
                  FloatToString(float_out,f, 15);
                  puts_UART(float_out); break;
                  
      }
    }
  }
  va_end(argp);
  return;
}

/*int main (void)
{
  int i=0;
  int k=1250;
  char st[128]="string";
  
	USART_init();
  while(1){
  _delay_ms(1000);
  PORTD |= 0x80;
	out_char('T');
	out_char('e');
	out_char('s');
	out_char('T');
  out_char(':');
  print_UART("This is integer= %d , the string = %s, the exa = %x ", i, st,k);
  i++;
  PORTD &= 0x7F;
	}
	return 0;
}*/
