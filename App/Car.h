#ifndef __CAR_H
#define __CAR_H
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "jy901s.h"
#include "control.h"
#include "encoder.h"
#include "pid.h"
#include "position.h"
#include "ros_protocol.h"

#define PWM_MAX 4900
#define PWM_MIN -4900


void Car_Init(void);


#endif

