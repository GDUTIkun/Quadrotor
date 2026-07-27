#include "Car.h"

#define CAR_START_STACK                 128
#define CAR_START_PRIORITY              1
TaskHandle_t car_start_handle;
void Car_Start(void* pv);

#define DATA_TASK_STACK                 128*2
#define DATA_TASK_STACK_PRIORITY        5
TaskHandle_t data_task_handle;
void Data_Task(void* pv);

#define CONTROL_TASK_STACK              128*2
#define CONTROL_TASK_STACK_PRIORITY     4
TaskHandle_t control_task_handle;
void Control_Task(void* pv);

#define NODE_TASK_STACK                 128
#define NODE_TASK_STACK_PRIORITY        3
TaskHandle_t node_task_handle;
void Node_Task(void* pv);


#define SHOW_TASK_STACK                 128*2
#define SHOW_TASK_STACK_PRIORITY        3
TaskHandle_t show_task_handle;
void Show_Task(void* pv);

#define DEBUG_TASK_STACK                 128
#define DEBUG_TASK_STACK_PRIORITY        2
TaskHandle_t debug_task_handle;
void Debug_Task(void* pv);

void Car_Init(void)
{
    xTaskCreate( (TaskFunction_t) Car_Start,
                (char *) "Car_Start", 
                (configSTACK_DEPTH_TYPE) CAR_START_STACK,
                (void *) NULL,
                (UBaseType_t) CAR_START_PRIORITY,
                (TaskHandle_t *) &car_start_handle );
    vTaskStartScheduler();
}

void Car_Start(void* pv)
{
    taskENTER_CRITICAL();
    
    xTaskCreate( (TaskFunction_t) Data_Task,
                (char *) "Data_Task", 
                (configSTACK_DEPTH_TYPE) DATA_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DATA_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &data_task_handle );
                
    xTaskCreate( (TaskFunction_t) Control_Task,
                (char *) "Control_Task", 
                (configSTACK_DEPTH_TYPE) CONTROL_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) CONTROL_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &control_task_handle );
                
    xTaskCreate( (TaskFunction_t) Show_Task,
                (char *) "Show_Task", 
                (configSTACK_DEPTH_TYPE) SHOW_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) SHOW_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &show_task_handle );
    xTaskCreate( (TaskFunction_t) Debug_Task,
                (char *) "Debug_Task", 
                (configSTACK_DEPTH_TYPE) DEBUG_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DEBUG_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &debug_task_handle );
                
    vTaskDelete(NULL);
                
    taskEXIT_CRITICAL();
}
/*===========================================
上面是任务初始化，下面是执行的任务
==============================================*/

double gyx, gyy, gyz, y, r, p;
double ax, ay, az;
float pwm_l;
float pwm_r;
short enl, enr;
float lodometry, rodometry;
static uint16_t data_task_tick;

void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        JY901S_Gyro(&gyx, &gyy, &gyz, &y, &r, &p);
        JY901S_Acc(&ax, &ay, &az);
        Get_Encoder(&enl, &enr);
        Get_Odemotry(enl, enr, &lodometry, &rodometry);
        RosProtocol_Poll(HAL_GetTick());
        RosProtocol_SendImu(ax, ay, az, gyx, gyy, gyz, y, p, r, HAL_GetTick());
        data_task_tick++;
        if (data_task_tick >= 100)
        {
            data_task_tick = 0;
            RosProtocol_SendStatus(0, 0, ROS_STATUS_READY, RosProtocol_GetErrorFlags(), HAL_GetTick());
        }
        xTaskNotifyGive(control_task_handle);//给控制任务通知
        xTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}



void Control_Task(void* pv)
{

    
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//等待通知
        Set_Motor(pwm_l, pwm_r);
//        xTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}

void Show_Task(void* pv)
{
    while(1)
    {

       vTaskDelay(50);
    }
}



void Debug_Task(void* pv)
{
    while(1)
    {
        vTaskDelay(100);
    }
}

void RosProtocol_CmdVelReceivedCallback(const RosCmdVel_t *cmd)
{
    if (cmd->enable == 0u)
    {
        pwm_l = 0.0f;
        pwm_r = 0.0f;
    }
}

void RosProtocol_CmdTimeoutCallback(void)
{
    pwm_l = 0.0f;
    pwm_r = 0.0f;
}


/*===================================================================================*/
//各种回调

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    else if(htim->Instance == TIM3)
    {
        HAL_TIM_Base_Stop_IT(&htim3);
        delayflag = 1;
    }
}

