#include "ros_protocol.h"

#include <string.h>

#define ROS_CMD_TIMEOUT_MS              500u
#define ROS_TX_TIMEOUT_MS               20u

typedef enum
{
    RX_WAIT_HEADER0 = 0,
    RX_WAIT_HEADER1,
    RX_MSG_ID,
    RX_SEQ,
    RX_LEN,
    RX_PAYLOAD,
    RX_CRC_LOW,
    RX_CRC_HIGH
} RosRxState_t;

static UART_HandleTypeDef *ros_uart;
static uint8_t ros_rx_byte;
static uint8_t ros_tx_seq;
static RosRxState_t rx_state;
static uint8_t rx_msg_id;
static uint8_t rx_seq;
static uint8_t rx_len;
static uint8_t rx_payload[ROS_PROTO_MAX_PAYLOAD_LEN];
static uint8_t rx_payload_index;
static uint8_t rx_crc_low;

static volatile RosCmdVel_t latest_cmd;
static volatile uint8_t cmd_valid;
static volatile uint8_t cmd_timeout_latched = 1u;
static volatile uint16_t protocol_error_flags = ROS_ERROR_CMD_TIMEOUT;
static volatile RosProtocolDebug_t protocol_debug;

static int16_t ros_i16_from_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void ros_put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static void ros_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void ros_put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8) & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static void ros_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static int16_t ros_double_to_i16(double v)
{
    if (v > 32767.0)
    {
        return 32767;
    }
    if (v < -32768.0)
    {
        return -32768;
    }
    return (int16_t)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

static double ros_normalize_deg(double deg)
{
    while (deg > 180.0)
    {
        deg -= 360.0;
    }
    while (deg < -180.0)
    {
        deg += 360.0;
    }
    return deg;
}

static uint16_t ros_crc16_ibm_update(uint16_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= data;
    for (i = 0; i < 8u; i++)
    {
        if ((crc & 0x0001u) != 0u)
        {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        }
        else
        {
            crc >>= 1;
        }
    }

    return crc;
}

static uint16_t ros_crc16_ibm(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000u;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        crc = ros_crc16_ibm_update(crc, data[i]);
    }

    return crc;
}

static void ros_parser_reset(void)
{
    rx_state = RX_WAIT_HEADER0;
    rx_payload_index = 0u;
}

static void ros_handle_cmd_vel(uint8_t seq, const uint8_t *payload)
{
    RosCmdVel_t cmd;

    cmd.v_mm_s = ros_i16_from_le(&payload[0]);
    cmd.w_mrad_s = ros_i16_from_le(&payload[2]);
    cmd.enable = payload[4] != 0u ? 1u : 0u;
    cmd.seq = seq;
    cmd.stamp_ms = HAL_GetTick();

    latest_cmd.v_mm_s = cmd.v_mm_s;
    latest_cmd.w_mrad_s = cmd.w_mrad_s;
    latest_cmd.enable = cmd.enable;
    latest_cmd.seq = cmd.seq;
    latest_cmd.stamp_ms = cmd.stamp_ms;
    protocol_debug.rx_cmd_vel_frames++;
    cmd_valid = 1u;
    cmd_timeout_latched = 0u;
    protocol_error_flags &= (uint16_t)~ROS_ERROR_CMD_TIMEOUT;
    RosProtocol_CmdVelReceivedCallback(&cmd);
}

static void ros_handle_frame(uint8_t msg_id, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if (msg_id == ROS_MSG_ID_CMD_VEL && len == 5u)
    {
        ros_handle_cmd_vel(seq, payload);
    }
}

static HAL_StatusTypeDef ros_send_frame(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[2u + 1u + 1u + 1u + ROS_PROTO_MAX_PAYLOAD_LEN + 2u];
    uint16_t crc;
    uint16_t frame_len;

    if (ros_uart == 0 || len > ROS_PROTO_MAX_PAYLOAD_LEN)
    {
        return HAL_ERROR;
    }

    frame[0] = ROS_PROTO_HEADER0;
    frame[1] = ROS_PROTO_HEADER1;
    frame[2] = msg_id;
    frame[3] = ros_tx_seq++;
    frame[4] = len;
    if (len > 0u && payload != 0)
    {
        memcpy(&frame[5], payload, len);
    }

    crc = ros_crc16_ibm(&frame[2], (uint16_t)(3u + len));
    frame[5u + len] = (uint8_t)(crc & 0xFFu);
    frame[6u + len] = (uint8_t)((crc >> 8) & 0xFFu);
    frame_len = (uint16_t)(7u + len);

    return HAL_UART_Transmit(ros_uart, frame, frame_len, ROS_TX_TIMEOUT_MS);
}

static void ros_parse_byte(uint8_t byte)
{
    uint8_t crc_buf[3u + ROS_PROTO_MAX_PAYLOAD_LEN];
    uint16_t expected_crc;
    uint16_t received_crc;

    protocol_debug.rx_bytes++;
    protocol_debug.last_rx_ms = HAL_GetTick();

    switch (rx_state)
    {
    case RX_WAIT_HEADER0:
        if (byte == ROS_PROTO_HEADER0)
        {
            rx_state = RX_WAIT_HEADER1;
        }
        break;

    case RX_WAIT_HEADER1:
        if (byte == ROS_PROTO_HEADER1)
        {
            rx_state = RX_MSG_ID;
        }
        else if (byte != ROS_PROTO_HEADER0)
        {
            rx_state = RX_WAIT_HEADER0;
        }
        break;

    case RX_MSG_ID:
        rx_msg_id = byte;
        rx_state = RX_SEQ;
        break;

    case RX_SEQ:
        rx_seq = byte;
        rx_state = RX_LEN;
        break;

    case RX_LEN:
        rx_len = byte;
        rx_payload_index = 0u;
        if (rx_len > ROS_PROTO_MAX_PAYLOAD_LEN)
        {
            protocol_debug.rx_len_errors++;
            ros_parser_reset();
        }
        else if (rx_len == 0u)
        {
            rx_state = RX_CRC_LOW;
        }
        else
        {
            rx_state = RX_PAYLOAD;
        }
        break;

    case RX_PAYLOAD:
        rx_payload[rx_payload_index++] = byte;
        if (rx_payload_index >= rx_len)
        {
            rx_state = RX_CRC_LOW;
        }
        break;

    case RX_CRC_LOW:
        rx_crc_low = byte;
        rx_state = RX_CRC_HIGH;
        break;

    case RX_CRC_HIGH:
        crc_buf[0] = rx_msg_id;
        crc_buf[1] = rx_seq;
        crc_buf[2] = rx_len;
        if (rx_len > 0u)
        {
            memcpy(&crc_buf[3], rx_payload, rx_len);
        }
        expected_crc = ros_crc16_ibm(crc_buf, (uint16_t)(3u + rx_len));
        received_crc = (uint16_t)rx_crc_low | ((uint16_t)byte << 8);
        if (received_crc == expected_crc)
        {
            protocol_debug.rx_frames++;
            protocol_debug.last_msg_id = rx_msg_id;
            protocol_debug.last_seq = rx_seq;
            protocol_debug.last_len = rx_len;
            ros_handle_frame(rx_msg_id, rx_seq, rx_payload, rx_len);
        }
        else
        {
            protocol_debug.rx_crc_errors++;
            protocol_error_flags |= ROS_ERROR_CRC_ERROR;
        }
        ros_parser_reset();
        break;

    default:
        ros_parser_reset();
        break;
    }
}

void RosProtocol_Init(UART_HandleTypeDef *huart)
{
    ros_uart = huart;
    ros_tx_seq = 0u;
    ros_parser_reset();
    cmd_valid = 0u;
    cmd_timeout_latched = 1u;
    protocol_error_flags = ROS_ERROR_CMD_TIMEOUT;
    memset((void *)&protocol_debug, 0, sizeof(protocol_debug));

    if (ros_uart != 0)
    {
        (void)HAL_UART_Receive_IT(ros_uart, &ros_rx_byte, 1u);
    }
}

void RosProtocol_Poll(uint32_t now_ms)
{
    uint32_t last_cmd_ms = latest_cmd.stamp_ms;

    if (cmd_valid != 0u && (uint32_t)(now_ms - last_cmd_ms) > ROS_CMD_TIMEOUT_MS)
    {
        latest_cmd.v_mm_s = 0;
        latest_cmd.w_mrad_s = 0;
        latest_cmd.enable = 0u;
        protocol_error_flags |= ROS_ERROR_CMD_TIMEOUT;
        if (cmd_timeout_latched == 0u)
        {
            cmd_timeout_latched = 1u;
            RosProtocol_CmdTimeoutCallback();
        }
    }
}

void RosProtocol_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ros_uart != 0 && huart == ros_uart)
    {
        ros_parse_byte(ros_rx_byte);
        (void)HAL_UART_Receive_IT(ros_uart, &ros_rx_byte, 1u);
    }
}

uint8_t RosProtocol_GetCmdVel(RosCmdVel_t *cmd)
{
    if (cmd == 0 || cmd_valid == 0u)
    {
        return 0u;
    }

    cmd->v_mm_s = latest_cmd.v_mm_s;
    cmd->w_mrad_s = latest_cmd.w_mrad_s;
    cmd->enable = latest_cmd.enable;
    cmd->seq = latest_cmd.seq;
    cmd->stamp_ms = latest_cmd.stamp_ms;
    return 1u;
}

void RosProtocol_GetDebug(RosProtocolDebug_t *debug)
{
    if (debug == 0)
    {
        return;
    }

    debug->rx_bytes = protocol_debug.rx_bytes;
    debug->rx_frames = protocol_debug.rx_frames;
    debug->rx_cmd_vel_frames = protocol_debug.rx_cmd_vel_frames;
    debug->rx_crc_errors = protocol_debug.rx_crc_errors;
    debug->rx_len_errors = protocol_debug.rx_len_errors;
    debug->last_msg_id = protocol_debug.last_msg_id;
    debug->last_seq = protocol_debug.last_seq;
    debug->last_len = protocol_debug.last_len;
    debug->last_rx_ms = protocol_debug.last_rx_ms;
}

uint32_t RosProtocol_GetLastCmdMs(void)
{
    return latest_cmd.stamp_ms;
}

uint16_t RosProtocol_GetErrorFlags(void)
{
    return protocol_error_flags;
}

void RosProtocol_ClearErrorFlags(uint16_t flags)
{
    protocol_error_flags &= (uint16_t)~flags;
}

HAL_StatusTypeDef RosProtocol_SendStatus(uint16_t voltage_mv,
                                         int16_t current_ma,
                                         uint8_t state,
                                         uint16_t error_flags,
                                         uint32_t stamp_ms)
{
    uint8_t payload[11];

    ros_put_u16(&payload[0], voltage_mv);
    ros_put_i16(&payload[2], current_ma);
    payload[4] = state;
    ros_put_u16(&payload[5], error_flags);
    ros_put_u32(&payload[7], stamp_ms);

    return ros_send_frame(ROS_MSG_ID_STATUS, payload, sizeof(payload));
}

HAL_StatusTypeDef RosProtocol_SendImu(double ax_g,
                                      double ay_g,
                                      double az_g,
                                      double gx_dps,
                                      double gy_dps,
                                      double gz_dps,
                                      double yaw_deg,
                                      double pitch_deg,
                                      double roll_deg,
                                      uint32_t stamp_ms)
{
    uint8_t payload[22];

    ros_put_i16(&payload[0], ros_double_to_i16(ax_g * 1000.0));
    ros_put_i16(&payload[2], ros_double_to_i16(ay_g * 1000.0));
    ros_put_i16(&payload[4], ros_double_to_i16(az_g * 1000.0));
    ros_put_i16(&payload[6], ros_double_to_i16(gx_dps * 100.0));
    ros_put_i16(&payload[8], ros_double_to_i16(gy_dps * 100.0));
    ros_put_i16(&payload[10], ros_double_to_i16(gz_dps * 100.0));
    ros_put_i16(&payload[12], ros_double_to_i16(ros_normalize_deg(yaw_deg) * 100.0));
    ros_put_i16(&payload[14], ros_double_to_i16(ros_normalize_deg(pitch_deg) * 100.0));
    ros_put_i16(&payload[16], ros_double_to_i16(ros_normalize_deg(roll_deg) * 100.0));
    ros_put_u32(&payload[18], stamp_ms);

    return ros_send_frame(ROS_MSG_ID_IMU, payload, sizeof(payload));
}

HAL_StatusTypeDef RosProtocol_SendWheelOdom(int32_t x_mm,
                                            int32_t y_mm,
                                            int32_t yaw_mrad,
                                            int16_t vx_mm_s,
                                            int16_t wz_mrad_s,
                                            int16_t left_mm_s,
                                            int16_t right_mm_s,
                                            uint32_t stamp_ms)
{
    uint8_t payload[24];

    ros_put_i32(&payload[0], x_mm);
    ros_put_i32(&payload[4], y_mm);
    ros_put_i32(&payload[8], yaw_mrad);
    ros_put_i16(&payload[12], vx_mm_s);
    ros_put_i16(&payload[14], wz_mrad_s);
    ros_put_i16(&payload[16], left_mm_s);
    ros_put_i16(&payload[18], right_mm_s);
    ros_put_u32(&payload[20], stamp_ms);

    return ros_send_frame(ROS_MSG_ID_WHEEL_ODOM, payload, sizeof(payload));
}

__weak void RosProtocol_CmdVelReceivedCallback(const RosCmdVel_t *cmd)
{
    (void)cmd;
}

__weak void RosProtocol_CmdTimeoutCallback(void)
{
}
