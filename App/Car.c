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
float lwheel_speed, rwheel_speed;
float lodometry, rodometry;
static uint16_t data_task_tick;
static volatile RosCmdVel_t debug_rx_cmd;
static volatile uint8_t debug_rx_pending;
static volatile uint8_t debug_timeout_pending;
static volatile uint32_t debug_rx_count;

void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        JY901S_Gyro(&gyx, &gyy, &gyz, &y, &r, &p);
        JY901S_Acc(&ax, &ay, &az);
        Get_Encoder(&enl, &enr);
        Get_WheelSpeed(enl, enr, &lwheel_speed, &rwheel_speed);
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


float v_target, gy_target;
float ff = 700;
void Control_Task(void* pv)
{
    RosCmdVel_t cmd;
    PID_Init(&l_velocity_pid,50000, 600, 0, 0);
    PID_Init(&r_velocity_pid,50000, 600, 0, 0);
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//等待通知
//        if (RosProtocol_GetCmdVel(&cmd) && cmd.enable)
//        {
//            // TODO: cmd.v_mm_s / cmd.w_mrad_s -> left/right wheel target -> PID -> pwm_l/pwm_r
//        }
//        else
//        {
//            pwm_l = 0.0f;
//            pwm_r = 0.0f;
//        }
        float pwm_l = -PID_Velocity(&l_velocity_pid, v_target-lwheel_speed);
        float pwm_r = -PID_Velocity(&r_velocity_pid, v_target-rwheel_speed)- v_target*ff;
        PID_Clamp(pwm_l, PWM_MIN, PWM_MAX);//限幅
        PID_Clamp(pwm_r, PWM_MIN, PWM_MAX);
        Set_Motor(pwm_l, pwm_r);//输入给电机
//        xTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}

void Show_Task(void* pv)
{
    while(1)
    {

       vTaskDelay(100);
    }
}



void Debug_Task(void* pv)
{
    RosCmdVel_t cmd;
    RosProtocolDebug_t debug;
    uint8_t has_cmd;
    uint8_t has_timeout;
    uint32_t rx_count;
    uint32_t last_print_ms;
    uint32_t now_ms;

    last_print_ms = HAL_GetTick();
    while(1)
    {
//        now_ms = HAL_GetTick();
//        has_cmd = 0u;
//        has_timeout = 0u;
//        rx_count = 0u;

//        taskENTER_CRITICAL();
//        if (debug_rx_pending != 0u)
//        {
//            cmd.v_mm_s = debug_rx_cmd.v_mm_s;
//            cmd.w_mrad_s = debug_rx_cmd.w_mrad_s;
//            cmd.enable = debug_rx_cmd.enable;
//            cmd.seq = debug_rx_cmd.seq;
//            cmd.stamp_ms = debug_rx_cmd.stamp_ms;
//            rx_count = debug_rx_count;
//            debug_rx_pending = 0u;
//            has_cmd = 1u;
//        }
//        if (debug_timeout_pending != 0u)
//        {
//            debug_timeout_pending = 0u;
//            has_timeout = 1u;
//        }
//        taskEXIT_CRITICAL();

//        if (has_cmd != 0u)
//        {
//            printf("[ROS RX] CMD_VEL seq=%u v=%dmm/s w=%dmrad/s en=%u t=%lums cnt=%lu\r\n",
//                   cmd.seq,
//                   cmd.v_mm_s,
//                   cmd.w_mrad_s,
//                   cmd.enable,
//                   (unsigned long)cmd.stamp_ms,
//                   (unsigned long)rx_count);
//        }
//        if (has_timeout != 0u)
//        {
//            printf("[ROS RX] CMD_TIMEOUT stop\r\n");
//        }
//        if ((uint32_t)(now_ms - last_print_ms) >= 1000u)
//        {
//            last_print_ms = now_ms;
//            RosProtocol_GetDebug(&debug);
//            printf("[ROS DBG] bytes=%lu frames=%lu cmd=%lu crc=%lu len=%lu last_id=0x%02X last_seq=%u last_len=%u last_rx=%lums\r\n",
//                   (unsigned long)debug.rx_bytes,
//                   (unsigned long)debug.rx_frames,
//                   (unsigned long)debug.rx_cmd_vel_frames,
//                   (unsigned long)debug.rx_crc_errors,
//                   (unsigned long)debug.rx_len_errors,
//                   debug.last_msg_id,
//                   debug.last_seq,
//                   debug.last_len,
//                   (unsigned long)debug.last_rx_ms);
//        }
        printf("%.5f,%.5f,%.5f\r\n", v_target, lwheel_speed, rwheel_speed);
        vTaskDelay(50);
    }
}

void RosProtocol_CmdVelReceivedCallback(const RosCmdVel_t *cmd)
{
    debug_rx_cmd.v_mm_s = cmd->v_mm_s;
    debug_rx_cmd.w_mrad_s = cmd->w_mrad_s;
    debug_rx_cmd.enable = cmd->enable;
    debug_rx_cmd.seq = cmd->seq;
    debug_rx_cmd.stamp_ms = cmd->stamp_ms;
    debug_rx_count++;
    debug_rx_pending = 1u;

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
    debug_timeout_pending = 1u;
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

