#include "stm_bridge/protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <type_traits>

namespace stm_bridge
{
namespace
{

constexpr std::uint16_t kCrcInit = 0x0000;
constexpr std::uint16_t kCrcPolyReversed = 0xA001;

template<typename T>
void append_little_endian(std::vector<std::uint8_t> & out, T value)
{
  using UnsignedT = std::make_unsigned_t<T>;
  auto raw = static_cast<UnsignedT>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFF));
  }
}

template<typename T>
T read_little_endian(const std::vector<std::uint8_t> & payload, std::size_t offset)
{
  using UnsignedT = std::make_unsigned_t<T>;
  UnsignedT raw = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    raw |= static_cast<UnsignedT>(payload[offset + i]) << (8 * i);
  }
  return static_cast<T>(raw);
}

std::int16_t clamp_int16(long value)
{
  return static_cast<std::int16_t>(std::max<long>(-32768, std::min<long>(32767, value)));
}

void require_payload_size(
  const std::vector<std::uint8_t> & payload, std::size_t expected,
  const char * name)
{
  if (payload.size() != expected) {
    throw std::runtime_error(
      std::string("bad ") + name + " payload length: " +
      std::to_string(payload.size()) + " != " + std::to_string(expected));
  }
}

}  // namespace

std::uint16_t crc16_ibm(const std::vector<std::uint8_t> & data)
{
  std::uint16_t crc = kCrcInit;
  for (const auto byte : data) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ kCrcPolyReversed);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::vector<std::uint8_t> build_frame(
  std::uint8_t msg_id, std::uint8_t seq,
  const std::vector<std::uint8_t> & payload)
{
  if (payload.size() > kMaxPayloadLen) {
    throw std::runtime_error("payload too large");
  }

  std::vector<std::uint8_t> body;
  body.reserve(3 + payload.size());
  body.push_back(msg_id);
  body.push_back(seq);
  body.push_back(static_cast<std::uint8_t>(payload.size()));
  body.insert(body.end(), payload.begin(), payload.end());

  const auto crc = crc16_ibm(body);

  std::vector<std::uint8_t> frame;
  frame.reserve(2 + body.size() + 2);
  frame.push_back(kHeader0);
  frame.push_back(kHeader1);
  frame.insert(frame.end(), body.begin(), body.end());
  append_little_endian<std::uint16_t>(frame, crc);
  return frame;
}

std::vector<std::uint8_t> encode_cmd_vel(double v_m_s, double w_rad_s, bool enable)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(5);
  append_little_endian<std::int16_t>(
    payload, clamp_int16(std::lround(v_m_s * 1000.0)));
  append_little_endian<std::int16_t>(
    payload, clamp_int16(std::lround(w_rad_s * 1000.0)));
  payload.push_back(enable ? 1 : 0);
  return payload;
}

ImuPayload decode_imu(const std::vector<std::uint8_t> & payload)
{
  require_payload_size(payload, 22, "IMU");
  return ImuPayload{
    read_little_endian<std::int16_t>(payload, 0),
    read_little_endian<std::int16_t>(payload, 2),
    read_little_endian<std::int16_t>(payload, 4),
    read_little_endian<std::int16_t>(payload, 6),
    read_little_endian<std::int16_t>(payload, 8),
    read_little_endian<std::int16_t>(payload, 10),
    read_little_endian<std::int16_t>(payload, 12),
    read_little_endian<std::int16_t>(payload, 14),
    read_little_endian<std::int16_t>(payload, 16),
    read_little_endian<std::uint32_t>(payload, 18)};
}

WheelOdomPayload decode_wheel_odom(const std::vector<std::uint8_t> & payload)
{
  require_payload_size(payload, 24, "wheel odom");
  return WheelOdomPayload{
    read_little_endian<std::int32_t>(payload, 0),
    read_little_endian<std::int32_t>(payload, 4),
    read_little_endian<std::int32_t>(payload, 8),
    read_little_endian<std::int16_t>(payload, 12),
    read_little_endian<std::int16_t>(payload, 14),
    read_little_endian<std::int16_t>(payload, 16),
    read_little_endian<std::int16_t>(payload, 18),
    read_little_endian<std::uint32_t>(payload, 20)};
}

StatusPayload decode_status(const std::vector<std::uint8_t> & payload)
{
  require_payload_size(payload, 11, "status");
  return StatusPayload{
    read_little_endian<std::uint16_t>(payload, 0),
    read_little_endian<std::int16_t>(payload, 2),
    read_little_endian<std::uint8_t>(payload, 4),
    read_little_endian<std::uint16_t>(payload, 5),
    read_little_endian<std::uint32_t>(payload, 7)};
}

FrameParser::FrameParser(std::size_t max_payload_len)
: max_payload_len_(max_payload_len)
{
}

std::vector<Frame> FrameParser::feed(const std::vector<std::uint8_t> & data)
{
  return feed(data.data(), data.size());
}

std::vector<Frame> FrameParser::feed(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<Frame> frames;

  while (true) {
    constexpr std::array<std::uint8_t, 2> header{kHeader0, kHeader1};
    auto header_it = std::search(
      buffer_.begin(), buffer_.end(),
      header.begin(), header.end());

    if (header_it == buffer_.end()) {
      dropped_bytes_ += buffer_.size();
      buffer_.clear();
      break;
    }

    if (header_it != buffer_.begin()) {
      dropped_bytes_ += static_cast<std::uint64_t>(header_it - buffer_.begin());
      buffer_.erase(buffer_.begin(), header_it);
    }

    if (buffer_.size() < 5) {
      break;
    }

    const auto payload_len = buffer_[4];
    if (payload_len > max_payload_len_) {
      ++dropped_bytes_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_len = 2 + 3 + payload_len + 2;
    if (buffer_.size() < frame_len) {
      break;
    }

    std::vector<std::uint8_t> body(buffer_.begin() + 2, buffer_.begin() + frame_len - 2);
    const std::uint16_t expected_crc =
      static_cast<std::uint16_t>(buffer_[frame_len - 2]) |
      (static_cast<std::uint16_t>(buffer_[frame_len - 1]) << 8);
    const auto actual_crc = crc16_ibm(body);
    if (actual_crc != expected_crc) {
      ++crc_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    frames.push_back(Frame{
      buffer_[2],
      buffer_[3],
      std::vector<std::uint8_t>(buffer_.begin() + 5, buffer_.begin() + frame_len - 2)});
    ++frames_received_;
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_len);
  }

  return frames;
}

std::uint64_t FrameParser::crc_errors() const
{
  return crc_errors_;
}

std::uint64_t FrameParser::dropped_bytes() const
{
  return dropped_bytes_;
}

std::uint64_t FrameParser::frames_received() const
{
  return frames_received_;
}

}  // namespace stm_bridge
