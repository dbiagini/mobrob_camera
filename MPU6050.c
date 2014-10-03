#include "MPU6050.h"


unsigned char buffer[14];

short  MPU_FIFO[6][11];
short Gx_offset=0,Gy_offset=0,Gz_offset=0;
float Acc1G_Values;
short MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;

/** Set clock source setting.
* An internal 8MHz oscillator, gyroscope based clock, or external sources can
* be selected as the MPU-60X0 clock source. *
* <pre>
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0 | Internal oscillator
* 1 | PLL with X Gyro reference
* 2 | PLL with Y Gyro reference
* 3 | PLL with Z Gyro reference
* 4 | PLL with external 32.768kHz reference
* 5 | PLL with external 19.2MHz reference
* 6 | Reserved
* 7 | Stops the clock and keeps the timing generator in reset
* </pre>
*/
void MPU_setClockSource(unsigned char source) {
    TWI_write_bits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU_setFullScaleGyroRange(unsigned char range) {
    TWI_write_bits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU_setFullScaleAccelRange(unsigned char range) {
    TWI_write_bits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
void MPU_setSleepEnabled(unsigned char enabled) {
    TWI_write_bit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
void MPU_setI2CMasterModeEnabled(unsigned char enabled) {
    TWI_write_bit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU_Calibrate_Gyros()
{
  unsigned char i;
	short temp[6];
	long	tempgx=0,tempgy=0,tempgz=0;
	long	tempax=0,tempay=0,tempaz=0;

  for(i=0;i<100;i++){
		MPU_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
    _delay_ms(20);
  
  }
  
  Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];
	/*tempax/=100;
	tempay/=100;
	tempaz/=100;
	Acc1G_Values= (float)(tempax+tempay+tempaz);*/
  return;

}

void MPU_initialize(void) {
	//short temp[6];
	//unsigned char i;
  MPU_setClockSource(MPU6050_CLOCK_PLL_XGYRO); 
  MPU_setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  MPU_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	
  MPU_setSleepEnabled(0); 
  MPU_setI2CMasterModeEnabled(0);	
  TWI_write_bit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1); //bypass enabled

  //interrupts stuff//
	/*TWI_write_bit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
	TWI_write_bit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
	TWI_write_bit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
	TWI_write_bit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
	
  TWI_write_bit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);*/

 /* for(i=0;i<10;i++){
	  delay_us(50);
	  MPU_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
	}
		*/																	 
  MPU_Calibrate_Gyros();
}

unsigned char MPU_testConnection(void) {

      
    TWI_read_bytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    if(buffer[0]==devAddr) return 0;
    else return 1;
}


void MPU_getMotion6(short* ax, short* ay, short* az, short* gx, short* gy, short* gz) {
  
    TWI_read_bytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((short)buffer[0]) << 8) | buffer[1];
    *ay = (((short)buffer[2]) << 8) | buffer[3];
    *az = (((short)buffer[4]) << 8) | buffer[5];
    *gx = ((((short)buffer[8]) << 8) | buffer[9])-Gx_offset;
    *gy = ((((short)buffer[10]) << 8) | buffer[11])-Gy_offset;
    *gz = ((((short)buffer[12]) << 8) | buffer[13])-Gz_offset;
}
 
/*//Converts the already acquired accelerometer data into 3D euler angles
void Get_Accel_Angles()
{
	ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)));
	ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)));	
}	*/
 
//Function to read the gyroscope rate data and convert it into degrees/s
void MPU_Get_Gyro_Rates(float* fx, float* fy, float* fz)
{ 
  short gx=0,gy=0,gz=0;
  TWI_read_bytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
  gx = ((((short)buffer[0]) << 8) | buffer[1])-Gx_offset;
  gy = ((((short)buffer[2]) << 8) | buffer[3])-Gy_offset;
  gz = ((((short)buffer[4]) << 8) | buffer[5])-Gz_offset;
  
	*fx =(float)gx/131; //in degrees
	*fy =(float)gy/131;
	*fz =(float)gz/131;
}
void MPU_Get_Accel_Values(float* fx, float* fy, float* fz)
{ 
  short gx=0,gy=0,gz=0;
  
    TWI_read_bytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    gx = (((short)buffer[0]) << 8) | buffer[1];
    gy = (((short)buffer[2]) << 8) | buffer[3];
    gz = (((short)buffer[4]) << 8) | buffer[5]; 

	*fx = (float)gx/0x4000; //in g
	*fy = (float)gy/0x4000;
	*fz = (float)gz/0x4000;
}
float MPU_1GValue(void)
{
	return Acc1G_Values;
}
