#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace stm_bridge
{

constexpr std::uint8_t kHeader0 = 0xAA;
constexpr std::uint8_t kHeader1 = 0x55;
constexpr std::uint8_t kMsgCmdVel = 0x01;
constexpr std::uint8_t kMsgImu = 0x81;
constexpr std::uint8_t kMsgWheelOdom = 0x82;
constexpr std::uint8_t kMsgStatus = 0x83;
constexpr std::size_t kMaxPayloadLen = 64;

struct Frame
{
  std::uint8_t msg_id{};
  std::uint8_t seq{};
  std::vector<std::uint8_t> payload;
};

struct ImuPayload
{
  std::int16_t ax_mg{};
  std::int16_t ay_mg{};
  std::int16_t az_mg{};
  std::int16_t gx_mdps{};
  std::int16_t gy_mdps{};
  std::int16_t gz_mdps{};
  std::int16_t yaw_cdeg{};
  std::int16_t pitch_cdeg{};
  std::int16_t roll_cdeg{};
  std::uint32_t stamp_ms{};
};

struct WheelOdomPayload
{
  std::int32_t x_mm{};
  std::int32_t y_mm{};
  std::int32_t yaw_mrad{};
  std::int16_t vx_mm_s{};
  std::int16_t wz_mrad_s{};
  std::int16_t left_mm_s{};
  std::int16_t right_mm_s{};
  std::uint32_t stamp_ms{};
};

struct StatusPayload
{
  std::uint16_t voltage_mv{};
  std::int16_t current_ma{};
  std::uint8_t state{};
  std::uint16_t error_flags{};
  std::uint32_t stamp_ms{};
};

std::uint16_t crc16_ibm(const std::vector<std::uint8_t> & data);

std::vector<std::uint8_t> build_frame(
  std::uint8_t msg_id, std::uint8_t seq,
  const std::vector<std::uint8_t> & payload);

std::vector<std::uint8_t> encode_cmd_vel(double v_m_s, double w_rad_s, bool enable);

ImuPayload decode_imu(const std::vector<std::uint8_t> & payload);

WheelOdomPayload decode_wheel_odom(const std::vector<std::uint8_t> & payload);

StatusPayload decode_status(const std::vector<std::uint8_t> & payload);

class FrameParser
{
public:
  explicit FrameParser(std::size_t max_payload_len = kMaxPayloadLen);

  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t> & data);

  std::uint64_t crc_errors() const;
  std::uint64_t dropped_bytes() const;
  std::uint64_t frames_received() const;

private:
  std::vector<std::uint8_t> buffer_;
  std::size_t max_payload_len_;
  std::uint64_t crc_errors_{};
  std::uint64_t dropped_bytes_{};
  std::uint64_t frames_received_{};
};

}  // namespace stm_bridge

