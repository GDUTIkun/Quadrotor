#include "stm_bridge/protocol.hpp"

#include <gtest/gtest.h>

#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace
{

std::vector<std::uint8_t> payload(std::initializer_list<std::uint8_t> bytes)
{
  return std::vector<std::uint8_t>(bytes);
}

std::string hex_string(const std::vector<std::uint8_t> & bytes)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(bytes[i]);
  }
  return stream.str();
}

template<typename T>
void append_le(std::vector<std::uint8_t> & out, T value)
{
  using UnsignedT = std::make_unsigned_t<T>;
  auto raw = static_cast<UnsignedT>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFF));
  }
}

}  // namespace

TEST(StmProtocol, Crc16KnownVector)
{
  const std::string text = "123456789";
  const std::vector<std::uint8_t> bytes(text.begin(), text.end());

  EXPECT_EQ(stm_bridge::crc16_ibm(bytes), 0xBB3D);
}

TEST(StmProtocol, CmdVelPayloadUnitsAndClamping)
{
  auto encoded = stm_bridge::encode_cmd_vel(1.234, -0.567, true);
  EXPECT_EQ(encoded, payload({0xD2, 0x04, 0xC9, 0xFD, 0x01}));

  encoded = stm_bridge::encode_cmd_vel(100.0, -100.0, false);
  EXPECT_EQ(encoded, payload({0xFF, 0x7F, 0x00, 0x80, 0x00}));
}

TEST(StmProtocol, DocumentedExampleFrames)
{
  std::vector<std::uint8_t> stop;
  append_le<std::int16_t>(stop, 0);
  append_le<std::int16_t>(stop, 0);
  stop.push_back(0);

  std::vector<std::uint8_t> forward;
  append_le<std::int16_t>(forward, 200);
  append_le<std::int16_t>(forward, 0);
  forward.push_back(1);

  std::vector<std::uint8_t> imu;
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 1000);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::int16_t>(imu, 0);
  append_le<std::uint32_t>(imu, 1000);

  std::vector<std::uint8_t> odom;
  append_le<std::int32_t>(odom, 0);
  append_le<std::int32_t>(odom, 0);
  append_le<std::int32_t>(odom, 0);
  append_le<std::int16_t>(odom, 0);
  append_le<std::int16_t>(odom, 0);
  append_le<std::int16_t>(odom, 0);
  append_le<std::int16_t>(odom, 0);
  append_le<std::uint32_t>(odom, 1000);

  std::vector<std::uint8_t> status;
  append_le<std::uint16_t>(status, 12000);
  append_le<std::int16_t>(status, 0);
  status.push_back(1);
  append_le<std::uint16_t>(status, 0);
  append_le<std::uint32_t>(status, 1000);

  EXPECT_EQ(
    hex_string(stm_bridge::build_frame(stm_bridge::kMsgCmdVel, 0, stop)),
    "AA 55 01 00 05 00 00 00 00 00 C1 99");
  EXPECT_EQ(
    hex_string(stm_bridge::build_frame(stm_bridge::kMsgCmdVel, 1, forward)),
    "AA 55 01 01 05 C8 00 00 00 01 F1 49");
  EXPECT_EQ(
    hex_string(stm_bridge::build_frame(stm_bridge::kMsgImu, 0, imu)),
    "AA 55 81 00 16 00 00 00 00 E8 03 00 00 00 00 00 00 00 00 00 00 "
    "00 00 E8 03 00 00 1D 51");
  EXPECT_EQ(
    hex_string(stm_bridge::build_frame(stm_bridge::kMsgWheelOdom, 0, odom)),
    "AA 55 82 00 18 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 E8 03 00 00 38 45");
  EXPECT_EQ(
    hex_string(stm_bridge::build_frame(stm_bridge::kMsgStatus, 0, status)),
    "AA 55 83 00 0B E0 2E 00 00 01 00 00 E8 03 00 00 85 A4");
}

TEST(StmProtocol, ParserHandlesNoiseHalfPacketsAndStickyPackets)
{
  const auto first = stm_bridge::build_frame(stm_bridge::kMsgCmdVel, 7, payload({97, 98, 99}));

  std::vector<std::uint8_t> imu_payload;
  for (std::int16_t value = 1; value <= 9; ++value) {
    append_le<std::int16_t>(imu_payload, value);
  }
  append_le<std::uint32_t>(imu_payload, 10);
  const auto second = stm_bridge::build_frame(stm_bridge::kMsgImu, 8, imu_payload);

  stm_bridge::FrameParser parser;
  std::vector<std::uint8_t> first_part{0x00, 'b', 'a', 'd'};
  first_part.insert(first_part.end(), first.begin(), first.begin() + 4);
  EXPECT_TRUE(parser.feed(first_part).empty());

  std::vector<std::uint8_t> second_part(first.begin() + 4, first.end());
  second_part.insert(second_part.end(), second.begin(), second.end());
  const auto frames = parser.feed(second_part);

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].msg_id, stm_bridge::kMsgCmdVel);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_EQ(frames[0].payload, payload({97, 98, 99}));
  const auto imu = stm_bridge::decode_imu(frames[1].payload);
  EXPECT_EQ(imu.gx_cdeg_s, 4);
  EXPECT_EQ(imu.gy_cdeg_s, 5);
  EXPECT_EQ(imu.gz_cdeg_s, 6);
  EXPECT_EQ(imu.stamp_ms, 10U);
  EXPECT_EQ(parser.dropped_bytes(), 4U);
}

TEST(StmProtocol, ParserDropsCrcErrorsAndResyncs)
{
  auto bad = stm_bridge::build_frame(stm_bridge::kMsgCmdVel, 1, payload({'b', 'a', 'd'}));
  bad.back() ^= 0xFF;
  const auto good = stm_bridge::build_frame(stm_bridge::kMsgCmdVel, 2, payload({'o', 'k'}));
  bad.insert(bad.end(), good.begin(), good.end());

  stm_bridge::FrameParser parser;
  const auto frames = parser.feed(bad);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].seq, 2);
  EXPECT_EQ(parser.crc_errors(), 1U);
}
