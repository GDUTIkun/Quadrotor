#ifndef __ROS_PROTOCOL_H
#define __ROS_PROTOCOL_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROS_PROTO_HEADER0               0xAAu
#define ROS_PROTO_HEADER1               0x55u
#define ROS_PROTO_MAX_PAYLOAD_LEN       64u

#define ROS_MSG_ID_CMD_VEL              0x01u
#define ROS_MSG_ID_IMU                  0x81u
#define ROS_MSG_ID_WHEEL_ODOM           0x82u
#define ROS_MSG_ID_STATUS               0x83u

#define ROS_STATUS_INIT                 0u
#define ROS_STATUS_READY                1u
#define ROS_STATUS_RUNNING              2u
#define ROS_STATUS_ESTOP                3u
#define ROS_STATUS_ERROR                4u

#define ROS_ERROR_CMD_TIMEOUT           (1u << 0)
#define ROS_ERROR_IMU_ERROR             (1u << 1)
#define ROS_ERROR_ENCODER_ERROR         (1u << 2)
#define ROS_ERROR_MOTOR_ERROR           (1u << 3)
#define ROS_ERROR_LOW_VOLTAGE           (1u << 4)
#define ROS_ERROR_CRC_ERROR             (1u << 5)

typedef struct
{
    int16_t v_mm_s;
    int16_t w_mrad_s;
    uint8_t enable;
    uint8_t seq;
    uint32_t stamp_ms;
} RosCmdVel_t;

typedef struct
{
    uint32_t rx_bytes;
    uint32_t rx_frames;
    uint32_t rx_cmd_vel_frames;
    uint32_t rx_crc_errors;
    uint32_t rx_len_errors;
    uint8_t last_msg_id;
    uint8_t last_seq;
    uint8_t last_len;
    uint32_t last_rx_ms;
} RosProtocolDebug_t;

void RosProtocol_Init(UART_HandleTypeDef *huart);
void RosProtocol_Poll(uint32_t now_ms);
void RosProtocol_RxCpltCallback(UART_HandleTypeDef *huart);

uint8_t RosProtocol_GetCmdVel(RosCmdVel_t *cmd);
void RosProtocol_GetDebug(RosProtocolDebug_t *debug);
uint32_t RosProtocol_GetLastCmdMs(void);
uint16_t RosProtocol_GetErrorFlags(void);
void RosProtocol_ClearErrorFlags(uint16_t flags);

HAL_StatusTypeDef RosProtocol_SendStatus(uint16_t voltage_mv,
                                         int16_t current_ma,
                                         uint8_t state,
                                         uint16_t error_flags,
                                         uint32_t stamp_ms);

HAL_StatusTypeDef RosProtocol_SendImu(double ax_g,
                                      double ay_g,
                                      double az_g,
                                      double gx_dps,
                                      double gy_dps,
                                      double gz_dps,
                                      double yaw_deg,
                                      double pitch_deg,
                                      double roll_deg,
                                      uint32_t stamp_ms);

HAL_StatusTypeDef RosProtocol_SendWheelOdom(int32_t x_mm,
                                            int32_t y_mm,
                                            int32_t yaw_mrad,
                                            int16_t vx_mm_s,
                                            int16_t wz_mrad_s,
                                            int16_t left_mm_s,
                                            int16_t right_mm_s,
                                            uint32_t stamp_ms);

void RosProtocol_CmdVelReceivedCallback(const RosCmdVel_t *cmd);
void RosProtocol_CmdTimeoutCallback(void);

#ifdef __cplusplus
}
#endif

#endif
