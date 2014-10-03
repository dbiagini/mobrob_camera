#include "acc_plot.h"

void IMU_print(unsigned short interval, float *RwAcc, float *RwEst)
{ 
  print_UART("%f",interval);  //microseconds since last sample, monitor this value to be < 10000, increase bitrate to print faster
  print_UART(",");    
  print_UART("%f",RwAcc[0]);  //Inclination X axis (as measured by accelerometer)
  print_UART(",");
  print_UART("%f",RwEst[0]);  //Inclination X axis (estimated / filtered)
  print_UART(",");    
  print_UART("%f",RwAcc[1]);  //Inclination Y axis (as measured by accelerometer)
  print_UART(",");
  print_UART("%f",RwEst[1]);  //Inclination Y axis (estimated / filtered)
  print_UART(",");    
  print_UART("%f",RwAcc[2]);  //Inclination Z axis (as measured by accelerometer)
  print_UART(",");
  print_UART("%f",RwEst[2]);  //Inclination Z axis (estimated / filtered)  
  print_UART("\r\n");
}
//Serial Acc version
void IMU_print_SerialAcc(float interval, float *RwAcc)
{ 
  print_UART("%f",interval);  //microseconds since last sample, monitor this value to be < 10000, increase bitrate to print faster
  print_UART(",");    
  print_UART(" X= %f",RwAcc[0]);  //Inclination X axis (as measured by accelerometer)

  print_UART(" Y= %f",RwAcc[1]);  //Inclination Y axis (as measured by accelerometer)
    
  print_UART(" Z= %f",RwAcc[2]);  //Inclination Z axis (as measured by accelerometer)

  print_UART("\r\n");

}
void IMU_simplot(float *data){
	
  print_UART("%f ", data[0]);  //Inclination X axis (as measured by accelerometer)

  print_UART("%f ", data[1]);  //Inclination Y axis (as measured by accelerometer)
    
  print_UART("%f ", data[2]);  //Inclination Z axis (as measured by accelerometer)

  print_UART("\r\n");
}
void IMU_simplot_int(int *data){
	
  print_UART("%d ", data[0]);  //Inclination X axis (as measured by accelerometer)

  print_UART("%d ", data[1]);  //Inclination Y axis (as measured by accelerometer)
    
  print_UART("%d ", data[2]);  //Inclination Z axis (as measured by accelerometer)

  print_UART("\r\n");
}
