#ifndef PID_H
#define PID_H

#include "math.h"

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float lastError;
    float lastLastError;
    float integral;
    float output;

}PID_ControllerTypeDef;


void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Clamp(float value, float min, float max);
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Position(PID_ControllerTypeDef *pid, float tx, float ty, float x, float y);
float PID_Gyro(PID_ControllerTypeDef *pid, float gyro);
//float PID_Turn(PID_ControllerTypeDef *pid, float yaw);
//float PID_Compute(PID_ControllerTypeDef *pid, float measurement);

extern PID_ControllerTypeDef velocity_pid;
extern PID_ControllerTypeDef gyro_pid;
//extern PID_ControllerTypeDef turn_pid;
//extern PID_ControllerTypeDef position_pid;
    
#endif //PID_H
