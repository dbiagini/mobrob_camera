#define F_CPU 8000000L
#include"serial.h"
#include"twi_mini.h"
#include"MPU6050.h"
#include<math.h>
#include"HMC5883l.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include"PWM.h"
//#include"matrix_math.h"

#define PI 3.14159265358979f
#define CT 125000.0f

#define LPF_SIZE 12
#define SERVO_K_X 1

#define SERVO_K_Y 1.6

void normalize3DVector(float* vector){
  static float R;  
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}
///function to low pass filter///

static float samp_vect[LPF_SIZE][3];
void lpf(float* vector){
	int i=0, j=0;
	float acc[3]={0.0,0.0,0.0,};
	//insert sample in filter//
	for(j=0; j<3; j++){
		for(i=1; i< LPF_SIZE; i++){
			samp_vect[i-1][j] = samp_vect[i][j];
		}
		samp_vect[LPF_SIZE-1][j] = vector[j];
	}
	//filter//
	for(j=0; j<3; j++){
		for(i=0; i< LPF_SIZE; i++){
			acc[j]+=samp_vect[i][j];
		}
		vector[j] = acc[j] / LPF_SIZE;
	}
	
}

float squared(float x){
  return x*x;
}
int overflows=0;
/*This function from a given vector and given rates from the gyroscope computes the estimation of the new position
*
*	inputs:
*	 in_pest - Position estimated from accel	
*	 in_rate - rates from Gyros
*	 in_dt - increment of time in seconds
*	outputs:
*	 o_est - estimated position filtered with gyro rates
*/
void fuse_withGyro(float* in_pest, float* in_rate, float in_dt, float* o_est){

         float PrjR[3];
         int signRzGyro=0;
         float angle[3]; 
         int i=0;

      if(in_pest && in_rate && o_est){

        if(fabs(in_pest[2]) < 0.1){   ///division
        for(i=0;i<=2;i++) o_est[i] = in_pest[i];
        }else{ 

        PrjR[0]= atan2(in_pest[0],in_pest[2]) * 180 / PI; ///calculate angle Ayz R on YZ (sums to Rx)
        PrjR[1]= atan2(in_pest[1],in_pest[2]) * 180 / PI; ///calculate angle Axz R on XZ (sums to Ry)
        PrjR[2]= atan2(in_pest[0],in_pest[1]) * 180 / PI; ///calculate angle Axy R on Xy (sums to Rz)

        for(i=0;i<3;i++)
        {
            angle[i]= PrjR[i]+ in_rate[i]* (in_dt); ///update the state in seconds
           
        }
        //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
        //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
        signRzGyro = ( cos(angle[1] * PI / 180) >=0 ) ? 1 : -1;

        o_est[0] = sin(angle[0] * PI / 180);
        o_est[0] /= sqrt( 1 + squared(cos(angle[0] * PI / 180)) * squared(tan(angle[1] * PI / 180)) );
        o_est[1] = sin(angle[1] * PI / 180);
        o_est[1] /= sqrt( 1 + squared(cos(angle[1] * PI / 180)) * squared(tan(angle[0] * PI / 180)) );  
        o_est[2] = signRzGyro * sqrt(1 - squared(o_est[0]) - squared(o_est[1]));
      }
    }

}

int main(void)
{
  //short values[6];
  //int ret_val=0;
  float vect[3]; 
  float accel[3];
 // float gyro[3];
  //float pure[3];
  float angles[3];
  uint16_t servo[3];
  //short mag_raw[3];
  int i;
  float Kg=0.5f;
  unsigned int timerT0=0, timerT1=0;
  float dT=0.0; //time increment in seconds
  float decl=0.089f; ///declination in radians

  USART_init();

  _delay_ms(1000);
  PORTD |= 0x80; ///turn led on
  cli();
  TCCR1B |=  /*(1 << CS11) | */ (1 << CS10); //start timer prescaled 8us (8MHz/64);
  // enable Timer1 overflow interrupt:
  TIMSK |= (1 << TOIE1);

  TWI_init_master();
  //Setup Servo and servo loop//
  servoResetPos();	
  servoStart();
  sei();
  /*for(i=0;i<3;i++)
  {
    accel[i]=0;
    vect[i]=0; ///zero the state
  }*/
   print_UART(" Servos set to middle position %d \r\n", SERVO_MID);
  _delay_ms(1000);
  if(MPU_testConnection()){
     print_UART(" MPU Test Connection Failed!! \r\n");
  }else{
    MPU_initialize();
    if(MAG_testConnection()){
       print_UART(" MAG Test Connection Failed!! \r\n");
    }else{
      float L_rate[3];
      float PrjRGyro[3];
      float mag_float[3];
      float heading=0;    
      float northEst[3];
      float northFuse[3];
      float heading_test=0;
      MAG_initialize();

      MPU_Get_Accel_Values(&accel[0], &accel[1], &accel[2]);
      normalize3DVector(accel);
      MAG_getHeading(&mag_float[0], &mag_float[1], &mag_float[2]);
      normalize3DVector(mag_float);
      for(i=0;i<3;i++){
        vect[i]=accel[i]; //init orientation  (-z)
        northEst[i]=mag_float[i];
      }
      
     TCNT1=0; //reset timer
     overflows=0;
     timerT0=TCNT1;
      
      while(1){     
        
        MPU_Get_Gyro_Rates(&L_rate[0], &L_rate[1], &L_rate[2]);
        MPU_Get_Accel_Values(&accel[1], &accel[0], &accel[2]); //has inverted x and y axes
        MAG_getHeading(&mag_float[0], &mag_float[1], &mag_float[2]);
        L_rate[1]=-L_rate[1];
        //normalize3DVector(L_rate);
        normalize3DVector(accel);
        normalize3DVector(mag_float);
        
       
        timerT1=TCNT1;
        //dT=abs(timerT1-timerT0);
	/* the time increment should be calculated in seconds */
        dT= (abs(timerT1>timerT0?timerT1-timerT0:((0xFFFF-timerT0)+timerT1)) + (overflows * 0xFFFF) ) / CT;
        //TCNT1=0x0;
        timerT0=TCNT1;
	overflows = 0;
        //timerT0=timerT1; 
        fuse_withGyro(vect,L_rate,dT,PrjRGyro);
       // fuse_withGyro(northEst,L_rate,dT, northFuse);
       
        
       for(i=0;i<=2;i++) { 
        vect[i] =(accel[i] + Kg* PrjRGyro[i]) / (1.0 + Kg);  //to check
        northEst[i]=mag_float[i];//(mag_float[i] + Kg* northFuse[i]) / (1.0 + Kg);  //to check
       }

       normalize3DVector(vect);
       angles[0]= atan2(vect[0], vect[2]) * 180 / PI; ///roll
       angles[1]= atan2(vect[1], vect[2]) * 180 / PI; ///pitch
       lpf(angles);

        // The tilt compensation algorithm till 40deg works//
        /*{
          float cosRoll,cosPitch,sinRoll,sinPitch,Xh,Yh;    
          float rollRadians = asin(accel[1]);
          float pitchRadians = asin(accel[0]);

          cosRoll=cos(angles[0]);
          cosPitch=cos(angles[1]);
          sinRoll=sin(angles[0]);
          sinPitch=sin(angles[1]);
          Xh = northEst[0] * cosPitch + northEst[1] * sinRoll* sinPitch- northEst[2]*cosRoll*sinPitch;
          Yh = northEst[1]* cosRoll + northEst[2] * sinRoll;
          heading=atan2(Yh,Xh);
         }*/

          //adapt the reference systems//
          
          //heading+= decl;
        
        // Correct for when signs are reversed.
         // if(heading < 0)  heading += 2*PI;
          // heading = heading * 180/PI;
        ///
        
        /* heading_test=atan2(northEst[1],northEst[0]);
        if(heading_test < 0)  heading_test += 2*PI;
        heading_test = heading_test * 180/PI;
        print_UART("MagHeading = %f , MagHeadingTest = %f \r\n", heading, heading_test);*/
          
       
        
         //angles[2]= heading; ///Yaw
	servo[0]= SERVO_MID + (SERVO_K_X * (angles[0]/180) * (SERVO_MAX-SERVO_MIN));
	servo[1]= SERVO_MID + (SERVO_K_Y * (angles[1]/180) * (SERVO_MAX-SERVO_MIN));
	servo[2]= SERVO_MID;

        servoSet(0, servo[0]);
        servoSet(1, servo[1]); 
 
        IMU_simplot_int(servo);

        //IMU_print(dT, angles, vect);
       //IMU_print_SerialAcc(dT, vect);
      }   
        
    
    }   
  }
  
  
  PORTD &= 0x7F;
    
 
  return 0;
}

ISR(TIMER1_OVF_vect)
{
    overflows++;
}


#if 0
int main(void)
{
  //short values[6];
  //int ret_val=0;
  float vect[3]; 
  float accel[3];
 // float gyro[3];
  //float pure[3];
  float angles[3];
  //short mag_raw[3];
  int i;
  float Kg=5.0f;
  unsigned short timerT0=0, timerT1=0, dT=0;
  float decl=0.089f; ///declination in radians

  USART_init();

  //_delay_ms(1000);
  //PORTD |= 0x80; ///turn led on
  print_UART(" Test Serial !! \r\n");
  TWI_init_master();
  sei();
  /*for(i=0;i<3;i++)
  {
    accel[i]=0;
    vect[i]=0; ///zero the state
  }*/

  print_UART(" TWI initialized !! \r\n");
  if(MPU_testConnection()){
     print_UART(" MPU Test Connection Failed!! \r\n");
  }else{
    print_UART(" MPU Test Connection Success!! \r\n");
    MPU_initialize();
    if(MAG_testConnection()){
       print_UART(" MAG Test Connection Failed!! \r\n");
    }else{
      
       print_UART(" MAG Test Connection Success!! \r\n");
    }   
        
    
  } 
  return 0;
}
#endif
#if 0
int main(void)
{
  
  short mag_raw[3];
  float mag_float[3];
  float decl=89/1000; ///declination in radians
  int i;
  unsigned short timerT0=0, timerT1=0, dT=0;
 
  USART_init();
  print_UART(" This is the test for MAG \r\n");

  _delay_ms(1000);
  PORTD |= 0x80; ///turn led on
  _delay_ms(100);

  print_UART(" Twi init master \r\n");
  TWI_init_master();
  sei();
  
  if(MPU_testConnection()){
     print_UART(" MPU Test Connection Failed!! \r\n");
  }else{
    MPU_initialize();
    if(MAG_testConnection()){
       print_UART(" MAG Test Connection Failed!! \r\n");
    }else{
      
      print_UART(" Mag Init \r\n");
      
    
      
      
     TCCR1B |= (1 << CS11); //start timer prescaled 1us (8MHz/8);
     TCNT1=0; //reset timer
     timerT0=TCNT1;
      
      while(1){
         
         
       MAG_getHeading(&mag_raw[0], &mag_raw[1], &mag_raw[2]);
       
        
        
       
        timerT1=TCNT1;
        dT=timerT1>timerT0?timerT1-timerT0:((0xFFFF-timerT0)+timerT1);
       // TCNT1=0; //reset timer
        timerT0=TCNT1=0; 
        for(i=0;i<3;i++) mag_float[i]=(float)mag_raw[i]; //copy
        normalize3DVector(mag_float);
        float heading=atan2(mag_float[1],mag_float[0]);
        heading+= decl;
        
        // Correct for when signs are reversed.
          if(heading < 0)
            heading += 2*PI;
           
        // Convert radians to degrees for readability.
        float headingDegrees = heading * 180/PI;
        print_UART("MagHeading = %f \r\n", headingDegrees);
        
 
        //IMU_print(dT, mag_float, mag_float);
      }    
        
    
    }   
  }
  
  
  PORTD &= 0x7F;
    
 
  return 0;
}
#endif
