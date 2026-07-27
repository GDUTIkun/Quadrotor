#ifndef __JY901S_H
#define __JY901S_H

#include "jy901s_register.h"
#include "MyI2C.h"


void JY901S_Acc(double* accx, double* accy, double* accz);
void JY901S_Gyro(double* gyx, double* gyy, double* gyz, double* yaw, double* roll, double* pitch);
void JY901S_Angle(double* roll, double* pitch, double* yaw);

#endif

