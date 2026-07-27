#include "jy901s.h"

void JY901S_Acc(double* accx, double* accy, double* accz)
{
    *accy = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x34)*0.0004883;
    *accx = -(double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x35)*0.0004883;
    *accz = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x36)*0.0004883;
}

void JY901S_Gyro(double* gyx, double* gyy, double* gyz, double* yaw, double* roll, double* pitch)
{
    *gyy = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x37)*0.061035;
    *gyx = -(double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x38)*0.061035;
    *gyz = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x39)*0.061035;
    *yaw += *gyz*0.01;
    *roll += *gyx*0.01;
    *pitch += *gyy*0.01;
}
//积分角速度获取角度

void JY901S_Angle(double* roll, double* pitch, double* yaw)
{
    *roll = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3d)*0.0054932;
    *pitch = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3e)*0.0054932;
    *yaw = (double)(int16_t)I2C_ReadReg(JY901SADDRESSS, 0x3f)*0.0054932;
}




