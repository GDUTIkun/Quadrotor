#include "position.h"


// gz单位是rad/s
void Get_LinearSpeed(short enl, short enr, double yaw, float* linear_speed)
{
    float wheell_speed, wheelr_speed;
    
    wheell_speed = (float)enl/ConvertParam*WheelCircumference;
    wheelr_speed = (float)enr/ConvertParam*WheelCircumference;
    *linear_speed = (wheelr_speed + wheell_speed) / 2.0f;
    
}

void Get_WheelSpeed(short enl, short enr, float *lwheel_speed, float *rwheel_speed)
{
    *lwheel_speed = (float)enl / ConvertParam * WheelCircumference;
    *rwheel_speed = (float)enr / ConvertParam * WheelCircumference;
}

void Get_Odemotry(short enl, short enr, float* lodometry, float *rodometry)
{
    float odemotry_lwheel_speed, odometry_rwheel_speed;

    Get_WheelSpeed(enl, enr, &odemotry_lwheel_speed, &odometry_rwheel_speed);
    *lodometry += odemotry_lwheel_speed*sampletime;
    *rodometry += odometry_rwheel_speed*sampletime;
}

void Get_Position(short enl, short enr, double yaw, float* x, float* y)
{
    float p_yaw;
    float linear_speed;
    
    p_yaw = yaw*0.01745;
    Get_LinearSpeed(enl, enr, yaw, &linear_speed);
    *y += linear_speed*cosf(p_yaw)*sampletime;
    *x -= linear_speed*sinf(p_yaw)*sampletime;
}



void InverseKinematics_differential(float linear_speed, double gz, float wheel_distance, float *lwheel_speed, float *rwheel_speed)
{
    *lwheel_speed = linear_speed - gz * wheel_distance / 2.0f;
    *rwheel_speed = linear_speed + gz * wheel_distance / 2.0f;
}
