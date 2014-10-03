#include"HMC5883l.h"

unsigned char buffer[6];

const short counts_per_milligauss[8]={  
    1370,  //+-0.88Ga
    1090,  //+-1.3Ga
    820,   //+-1.9Ga
    660,   //+-2.5Ga
    440,   //+-4.0Ga
    390,   //+-4.7Ga
    330,   //+-5.6Ga
    230	   //+-8.1Ga
  };


void MAG_initialize() {
unsigned char comm=0;

    // write CONFIG_A register
  
  comm=(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));

  TWI_write_bytes(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 1 , &comm);
  // write CONFIG_B register
  comm=HMC5883L_GAIN_1090 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
  TWI_write_bytes(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 1 , &comm);
    
  //write MODE register
  comm=HMC5883L_MODE_SINGLE<<(HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
  TWI_write_bytes(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 1, &comm);
}


int MAG_testConnection(void){
    TWI_read_bytes(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, 3, buffer);
    if(buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3') return 0;
    else return 1;
}

void MAG_getHeading(float *x, float *y, float *z) {
    unsigned char comm=  HMC5883L_MODE_SINGLE<<(HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
    TWI_read_bytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
    TWI_write_bytes(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 1, &comm); //issue read command
    *x = (float)((((short)buffer[0]) << 8) | buffer[1])/1090;
    *y = (float)((((short)buffer[4]) << 8) | buffer[5])/1090;
    *z = (float)((((short)buffer[2]) << 8) | buffer[3])/1090;
}
