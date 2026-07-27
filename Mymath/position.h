#ifndef __POSITION_H
#define __POSITION_H

#include "jy901s.h"
#include "math.h"

#define ConvertParam (13.0f*30.0f*4.0f/100)  // 从脉冲转到转速的转换参数 转速 = 脉冲数 / (线数 * 减速比 * 4) (转/10毫秒)
#define WheelDistance 0.023156f // 单位：m
#define WheelRadius 0.00325f // 单位：m
#define WheelCircumference 0.02042f // 单位：m
#define sampletime 0.01f // 单位：s

void Get_LinearSpeed(short enl, short enr, double yaw, float* linear_speed);
void Get_WheelSpeed(short enl, short enr, float *lwheel_speed, float *rwheel_speed);
void Get_Position(short enl, short enr, double yaw, float* x, float* y);
void InverseKinematics_differential(float linear_speed, double gz, float wheel_distance, float *lwheel_speed, float *rwheel_speed);
void Get_Odemotry(short enl, short enr, float* lodometry, float *rodometry);

#endif
