#include "serial.h"
void IMU_print(unsigned short interval, float *RwAcc, float *RwEst);
void IMU_print_SerialAcc(float interval, float *RwAcc);
void IMU_simplot(float *data);
void IMU_simplot_int(int *data);
