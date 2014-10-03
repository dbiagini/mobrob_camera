

#include"twi_mini.h"

//#include"serial.h"


static unsigned char read=1, write=0; //address=0x68

/*int main(void)
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
*/
void TWI_init_master(void) // Function to initialize master
{
 // TWBR=0x0B; // Bit rate 400 KHz
  TWSR&=0x0;//(1<<TWPS1)|(1<<TWPS0); // Setting prescalar bits
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
  // SCL freq= F_CPU/(16+2(TWBR).4^TWPS)
}

unsigned char TWI_start(void)
{
  // Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
  TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while(!(TWCR & (1<<TWINT))); // Wait till start condition is transmitted
  //while((TWSR & 0xF8)!= 0x08);//Might get stuck here // Check for the acknowledgement
  if(((TWSR & 0xF8)!= START_ACK)&&((TWSR & 0xF8)!= START_ACK_R)) {
    TWI_error(TWSR & 0xF8);//handle error
    return 1;
  }
  return 0;
}

unsigned char TWI_write_address(unsigned char SLA_X)
{
  TWDR=SLA_X; // Address and write/read instruction
  TWCR=(1<<TWINT)|(1<<TWEN); // Clear TWI interrupt flag,Enable TWI
  while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted

  //while(((TWSR & 0xF8)!= SLA_R_ACK)&&((TWSR & 0xF8)!= SLA_W_ACK)); //Might get stuck // Check for the acknoledgement
  if(((TWSR & 0xF8)!= SLA_R_ACK)&&((TWSR & 0xF8)!= SLA_W_ACK)) {
    TWI_error(TWSR & 0xF8);//handle error
    return 1;
  }
  return 0;

}

unsigned char TWI_write_byte(unsigned char data)
{
  TWDR=data; // put data in TWDR
  TWCR=(1<<TWINT)|(1<<TWEN); // Clear TWI interrupt flag,Enable TWI
  while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted

  //while((TWSR & 0xF8) != 0x28);//Might Get Stuck // Check for the acknoledgement
  if(((TWSR & 0xF8)!= DATA_R_ACK)&&((TWSR & 0xF8)!= DATA_W_ACK)) {
    TWI_error(TWSR & 0xF8);//handle error
    return 1;
  }
  return 0;

}

unsigned char TWI_read_byte(unsigned char ack)
{
  unsigned char data;
  TWCR = (1<<TWINT)|(ack<<TWEA)|(1<<TWEN); //Clear TWI interrupt flag,Enable TWI
  while (!(TWCR & (1<<TWINT))); // wait for status bits to indicate something was transmitted on the bus
  data = TWDR;   //load the returned data
  if (((TWSR & 0xF8)!= DATA_R_ACK)&&((TWSR & 0xF8)!= DATA_R_NACK)){ //check to see if the data was receved and return with a NACK, 0x58
    TWI_error(TWSR & 0xF8); //return, error
  }
  return data;

}
unsigned char TWI_read_bytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data){
  //this is the only function that returns 0 in case of failure
  unsigned char  count = 0;
	if(TWI_start()) return 0;
  if(TWI_write_address((dev<<1)|write)) return 0;
  if(TWI_write_byte(reg)) return 0; //write register
  if(TWI_start()) return 0; //repeated start
	if(TWI_write_address((dev<<1)|read)) return 0;//ready to read
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1) data[count]= TWI_read_byte(1); ///read byte and nack
		 else  data[count]=TWI_read_byte(0);	 ///read byte and ack
	}
  TWI_stop();
  return count;
}

unsigned char  TWI_write_bytes(unsigned char  dev, unsigned char  reg, unsigned char  length, unsigned char * data){
  
 	unsigned char count = 0;
	if(TWI_start()) return 1;
	if(TWI_write_address((dev<<1)|write)) return 1;   
	if(TWI_write_byte(reg)) return 1; //write register	  
	for(count=0;count<length;count++){
    if(TWI_write_byte(data[count])) return 1; //write data
	}
	TWI_stop();

  return 0; 
}
unsigned char TWI_write_bits(unsigned char  dev,unsigned char  reg,unsigned char  bitStart,unsigned char  length,unsigned char  data)
{

    unsigned char  b=0;;
    if(TWI_read_bytes(dev, reg, 1, &b)){  //should return count in case of success
      unsigned char mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
      data <<= (8 - length);
      data >>= (7 - bitStart);
      b &= mask;
      b |= data;
      return TWI_write_bytes(dev, reg, 1, &b); //returns 1 in case of success 0 in case of failure
    }else return 0;
   
}

unsigned char  TWI_write_bit(unsigned char  dev, unsigned char  reg, unsigned char  bitNum, unsigned char  data){
    unsigned char  b;
    if(TWI_read_bytes(dev, reg, 1, &b)){  //should return count in case of success
      b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
      return TWI_write_bytes(dev, reg, 1, &b); //returns 1 in case of success 0 in case of failure
     }else return 0;
}

void TWI_stop(void)
{
  // Clear TWI interrupt flag, Put stop condition on SDA, Enable TWI
  TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
  while(!(TWCR & (1<<TWSTO))); //not on the chart // Wait till stop     condition is transmitted
 return;
}

unsigned char TWI_error(unsigned char in_ERR){
///very basic error handling
  ////print error code
#ifdef __DEBUG__
 print_UART("error= %x ", in_ERR);
#endif
return 1;
}

