#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 独立的八方向低重复航点 ROS 2 节点，不依赖也不修改原 PathPlanner。

namespace {
constexpr int kWidth = 9;
constexpr int kHeight = 7;
constexpr int kCellCount = kWidth * kHeight;
constexpr int kStart = 8;  // A9B1
constexpr int kBlockedCount = 3;
constexpr int kFreeCount = kCellCount - kBlockedCount;
constexpr int kMaxExtraWaypoints = 18;
constexpr std::size_t kBeamWidth = 30000;
constexpr std::uint32_t kTurnPenalty45 = 200;
constexpr std::uint32_t kTurnPenalty90 = 500;
constexpr std::uint32_t kTurnPenalty135 = 1000;
constexpr std::uint32_t kTurnPenalty180 = 1000;
constexpr std::uint8_t kNoDirection = 8;
constexpr int kScreenFirstCenterX = 90;
constexpr int kScreenFirstCenterY = 30;
constexpr int kScreenCellSize = 60;

struct State {
  std::uint64_t visited{};
  std::array<std::uint8_t, kFreeCount + kMaxExtraWaypoints> path{};
  std::uint32_t distance_milli{};
  std::uint32_t turn_cost_milli{};
  int score{};
  std::uint8_t current{};
  std::uint8_t direction{kNoDirection};
  std::uint8_t length{};
  std::uint8_t covered{};
};

struct StateKey {
  std::uint64_t visited{};
  std::uint8_t current{};
  std::uint8_t direction{};

  bool operator==(const StateKey & other) const noexcept
  {
    return visited == other.visited && current == other.current && direction == other.direction;
  }
};

struct StateKeyHash {
  std::size_t operator()(const StateKey & key) const noexcept
  {
    const std::uint64_t mixed = key.visited ^
      (static_cast<std::uint64_t>(key.current) * 0x9e3779b97f4a7c15ULL) ^
      (static_cast<std::uint64_t>(key.direction) * 0xbf58476d1ce4e5b9ULL);
    return static_cast<std::size_t>(mixed ^ (mixed >> 32U));
  }
};

int cellId(int x, int y)
{
  return (y - 1) * kWidth + (x - 1);
}

int cellX(int id)
{
  return id % kWidth + 1;
}

int cellY(int id)
{
  return id / kWidth + 1;
}

std::string cellCode(int id)
{
  return "A" + std::to_string(cellX(id)) + "B" + std::to_string(cellY(id));
}

int parseCell(std::string code)
{
  code.erase(
    std::remove_if(code.begin(), code.end(), [](unsigned char ch) {return std::isspace(ch);}),
    code.end());
  std::transform(code.begin(), code.end(), code.begin(),
    [](unsigned char ch) {return static_cast<char>(std::toupper(ch));});

  if (code.size() != 4U || code[0] != 'A' || code[2] != 'B' ||
    !std::isdigit(static_cast<unsigned char>(code[1])) ||
    !std::isdigit(static_cast<unsigned char>(code[3])))
  {
    throw std::invalid_argument("非法方格代码: " + code + "，正确格式如 A4B3");
  }

  const int x = code[1] - '0';
  const int y = code[3] - '0';
  if (x < 1 || x > kWidth || y < 1 || y > kHeight) {
    throw std::out_of_range("方格超出 9x7 范围: " + code);
  }
  return cellId(x, y);
}

std::array<int, 8> neighbors(int id)
{
  const int x = cellX(id);
  const int y = cellY(id);
  return {
    x > 1 ? id - 1 : -1,
    x < kWidth ? id + 1 : -1,
    y > 1 ? id - kWidth : -1,
    y < kHeight ? id + kWidth : -1,
    x > 1 && y > 1 ? id - kWidth - 1 : -1,
    x < kWidth && y > 1 ? id - kWidth + 1 : -1,
    x > 1 && y < kHeight ? id + kWidth - 1 : -1,
    x < kWidth && y < kHeight ? id + kWidth + 1 : -1
  };
}

std::uint64_t validateBlocked(const std::vector<std::string> & codes)
{
  if (codes.size() != kBlockedCount) {
    throw std::invalid_argument("必须恰好输入 3 个禁飞格");
  }

  std::uint64_t blocked = 0;
  int min_x = kWidth;
  int max_x = 1;
  int min_y = kHeight;
  int max_y = 1;
  for (const std::string & code : codes) {
    const int id = parseCell(code);
    const std::uint64_t bit = 1ULL << id;
    if ((blocked & bit) != 0U) {
      throw std::invalid_argument("禁飞格重复: " + cellCode(id));
    }
    blocked |= bit;
    min_x = std::min(min_x, cellX(id));
    max_x = std::max(max_x, cellX(id));
    min_y = std::min(min_y, cellY(id));
    max_y = std::max(max_y, cellY(id));
  }

  const int width = max_x - min_x + 1;
  const int height = max_y - min_y + 1;
  if (!((width == 3 && height == 1) || (width == 1 && height == 3))) {
    throw std::invalid_argument("3 个禁飞格必须组成连续的 1x3 或 3x1 矩形");
  }
  if ((blocked & (1ULL << kStart)) != 0U) {
    throw std::invalid_argument("固定起点 A9B1 不能是禁飞格");
  }
  return blocked;
}

speed_t baudToTermios(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw std::invalid_argument(
              "serial_baud 仅支持 9600/19200/38400/57600/115200/230400");
  }
}

int serialBytesToCellId(std::uint8_t a_code, std::uint8_t b_code)
{
  if ((a_code & 0xF0U) != 0xA0U || (a_code & 0x0FU) < 1U ||
    (a_code & 0x0FU) > static_cast<std::uint8_t>(kWidth))
  {
    throw std::out_of_range("横向坐标字节必须在 0xA1～0xA9 范围内");
  }
  if ((b_code & 0xF0U) != 0xB0U || (b_code & 0x0FU) < 1U ||
    (b_code & 0x0FU) > static_cast<std::uint8_t>(kHeight))
  {
    throw std::out_of_range("纵向坐标字节必须在 0xB1～0xB7 范围内");
  }
  const int x = static_cast<int>(a_code & 0x0FU);
  const int y = static_cast<int>(b_code & 0x0FU);
  return cellId(x, y);
}

int screenX(int id)
{
  return kScreenFirstCenterX + (cellX(id) - 1) * kScreenCellSize;
}

int screenY(int id)
{
  return kScreenFirstCenterY + (kHeight - cellY(id)) * kScreenCellSize;
}

std::string bytesToHex(const std::uint8_t * data, std::size_t size)
{
  std::ostringstream result;
  result << std::uppercase << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    if (index != 0U) {
      result << ' ';
    }
    result << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return result.str();
}

int evaluate(const State & state, std::uint64_t blocked)
{
  const std::uint64_t free_mask = ((1ULL << kCellCount) - 1ULL) & ~blocked;
  std::uint64_t unvisited = free_mask & ~state.visited;
  if (unvisited == 0U) {
    return std::numeric_limits<int>::max();
  }

  int components = 0;
  int isolated = 0;
  int leaves = 0;
  std::uint64_t pending_cells = unvisited;
  while (pending_cells != 0U) {
    const int seed = __builtin_ctzll(pending_cells);
    ++components;
    std::uint64_t component_seen = 1ULL << seed;
    std::queue<int> pending;
    pending.push(seed);
    pending_cells &= ~(1ULL << seed);
    while (!pending.empty()) {
      const int value = pending.front();
      pending.pop();
      for (const int next : neighbors(value)) {
        if (next < 0 || (pending_cells & (1ULL << next)) == 0U) {
          continue;
        }
        pending_cells &= ~(1ULL << next);
        component_seen |= 1ULL << next;
        pending.push(next);
      }
    }
    (void)component_seen;
  }

  std::uint64_t scan = unvisited;
  while (scan != 0U) {
    const int value = __builtin_ctzll(scan);
    scan &= scan - 1U;
    int available_degree = 0;
    for (const int next : neighbors(value)) {
      if (next >= 0 && ((unvisited & (1ULL << next)) != 0U || next == state.current)) {
        ++available_degree;
      }
    }
    isolated += available_degree == 0 ? 1 : 0;
    leaves += available_degree == 1 ? 1 : 0;
  }

  int onward_degree = 0;
  for (const int next : neighbors(state.current)) {
    if (next >= 0 && (unvisited & (1ULL << next)) != 0U) {
      ++onward_degree;
    }
  }

  // 每覆盖一个新方格至少还要飞行 1 格，以此构造剩余路程的乐观下界。
  // 评分首先偏向总路程下界较短的状态，连通性和死角只用于引导束搜索。
  const int remaining = kFreeCount - static_cast<int>(state.covered);
  const int cost_lower_bound = static_cast<int>(state.distance_milli + state.turn_cost_milli) +
    remaining * 1000;
  return -cost_lower_bound * 100 - components * 5000 - isolated * 20000 -
         leaves * 300 - onward_degree * 20;
}

int moveCost(int from, int to)
{
  const bool diagonal = cellX(from) != cellX(to) && cellY(from) != cellY(to);
  return diagonal ? 1414 : 1000;
}

int directionIndex(int from, int to)
{
  const int delta_x = cellX(to) - cellX(from);
  const int delta_y = cellY(to) - cellY(from);
  if (delta_x == 1 && delta_y == 0) {return 0;}
  if (delta_x == 1 && delta_y == 1) {return 1;}
  if (delta_x == 0 && delta_y == 1) {return 2;}
  if (delta_x == -1 && delta_y == 1) {return 3;}
  if (delta_x == -1 && delta_y == 0) {return 4;}
  if (delta_x == -1 && delta_y == -1) {return 5;}
  if (delta_x == 0 && delta_y == -1) {return 6;}
  if (delta_x == 1 && delta_y == -1) {return 7;}
  throw std::logic_error("无法计算非相邻航点的方向");
}

std::uint32_t turnPenalty(std::uint8_t previous, std::uint8_t next)
{
  if (previous == kNoDirection) {
    return 0;
  }
  const int difference = std::abs(static_cast<int>(previous) - static_cast<int>(next));
  const int turn_steps = std::min(difference, 8 - difference);
  switch (turn_steps) {
    case 0: return 0;
    case 1: return kTurnPenalty45;
    case 2: return kTurnPenalty90;
    case 3: return kTurnPenalty135;
    case 4: return kTurnPenalty180;
    default: throw std::logic_error("无效的转弯角度");
  }
}

std::uint32_t totalCost(const State & state)
{
  return state.distance_milli + state.turn_cost_milli;
}

std::uint32_t theoreticalMinimumDistance(std::uint64_t blocked)
{
  const int start_color = (cellX(kStart) + cellY(kStart)) % 2;
  int start_color_cells = 0;
  int other_color_cells = 0;
  for (int id = 0; id < kCellCount; ++id) {
    if ((blocked & (1ULL << id)) != 0U) {
      continue;
    }
    if ((cellX(id) + cellY(id)) % 2 == start_color) {
      ++start_color_cells;
    } else {
      ++other_color_cells;
    }
  }

  // 水平/垂直移动会切换棋盘颜色，斜线移动保持颜色。
  // 固定从起点颜色开始时，由两种颜色数量差可推出至少需要多少段斜线。
  const int minimum_diagonals = std::max(
    {0, start_color_cells - other_color_cells - 1, other_color_cells - start_color_cells});
  const int segment_count = kFreeCount - 1;
  return static_cast<std::uint32_t>(
    (segment_count - minimum_diagonals) * 1000 + minimum_diagonals * 1414);
}

std::vector<int> planLowRepeat(std::uint64_t blocked)
{
  const std::uint32_t global_distance_lower_bound = theoreticalMinimumDistance(blocked);
  State initial;
  initial.visited = 1ULL << kStart;
  initial.current = kStart;
  initial.length = 1;
  initial.covered = 1;
  initial.distance_milli = 0;
  initial.turn_cost_milli = 0;
  initial.direction = kNoDirection;
  initial.path[0] = kStart;
  initial.score = evaluate(initial, blocked);

  std::vector<State> beam{initial};
  State best;
  bool found = false;
  for (int target_length = 2;
    target_length <= kFreeCount + kMaxExtraWaypoints; ++target_length)
  {
    std::vector<State> candidates;
    candidates.reserve(beam.size() * 3U);
    std::unordered_map<StateKey, std::size_t, StateKeyHash> candidate_index;
    candidate_index.reserve(beam.size() * 4U);

    for (const State & state : beam) {
      for (const int next : neighbors(state.current)) {
        if (next < 0 || (blocked & (1ULL << next)) != 0U) {
          continue;
        }
        State candidate = state;
        const std::uint8_t next_direction = static_cast<std::uint8_t>(
          directionIndex(state.current, next));
        candidate.current = static_cast<std::uint8_t>(next);
        candidate.path[candidate.length++] = static_cast<std::uint8_t>(next);
        candidate.distance_milli += static_cast<std::uint32_t>(moveCost(state.current, next));
        candidate.turn_cost_milli += turnPenalty(state.direction, next_direction);
        candidate.direction = next_direction;
        const std::uint64_t bit = 1ULL << next;
        if ((candidate.visited & bit) == 0U) {
          candidate.visited |= bit;
          ++candidate.covered;
        }

        if (candidate.covered == kFreeCount) {
          if (totalCost(candidate) == global_distance_lower_bound) {
            return std::vector<int>(
              candidate.path.begin(), candidate.path.begin() + candidate.length);
          }
          if (!found || totalCost(candidate) < totalCost(best) ||
            (totalCost(candidate) == totalCost(best) &&
            candidate.distance_milli < best.distance_milli) ||
            (totalCost(candidate) == totalCost(best) &&
            candidate.distance_milli == best.distance_milli && candidate.length < best.length))
          {
            best = candidate;
            found = true;
          }
          continue;
        }

        const std::uint32_t optimistic_cost = totalCost(candidate) +
          static_cast<std::uint32_t>(kFreeCount - candidate.covered) * 1000U;
        if (found && optimistic_cost >= totalCost(best)) {
          continue;
        }

        const StateKey key{candidate.visited, candidate.current, candidate.direction};
        const auto existing = candidate_index.find(key);
        if (existing != candidate_index.end()) {
          State & retained = candidates[existing->second];
          if (totalCost(candidate) < totalCost(retained) ||
            (totalCost(candidate) == totalCost(retained) &&
            candidate.distance_milli < retained.distance_milli))
          {
            candidate.score = evaluate(candidate, blocked);
            retained = std::move(candidate);
          }
          continue;
        }
        candidate.score = evaluate(candidate, blocked);
        candidate_index.emplace(key, candidates.size());
        candidates.push_back(std::move(candidate));
      }
    }

    if (candidates.empty()) {
      break;
    }
    if (candidates.size() > kBeamWidth) {
      std::nth_element(
        candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(kBeamWidth),
        candidates.end(), [](const State & left, const State & right) {
          return left.score > right.score;
        });
      candidates.resize(kBeamWidth);
    }
    beam = std::move(candidates);
  }
  if (found) {
    return std::vector<int>(best.path.begin(), best.path.begin() + best.length);
  }
  throw std::runtime_error("在搜索上限内没有找到覆盖路径，请增大 kMaxExtraWaypoints");
}

void checkPath(const std::vector<int> & path, std::uint64_t blocked)
{
  if (path.empty() || path.front() != kStart) {
    throw std::runtime_error("内部错误：路径起点不正确");
  }
  std::uint64_t covered = 0;
  for (std::size_t index = 0; index < path.size(); ++index) {
    const int value = path[index];
    if ((blocked & (1ULL << value)) != 0U) {
      throw std::runtime_error("内部错误：路径经过禁飞格");
    }
    covered |= 1ULL << value;
    if (index > 0U) {
      const int delta_x = std::abs(cellX(value) - cellX(path[index - 1U]));
      const int delta_y = std::abs(cellY(value) - cellY(path[index - 1U]));
      if (std::max(delta_x, delta_y) != 1) {
        throw std::runtime_error("内部错误：出现不相邻航点");
      }
    }
  }
  if (__builtin_popcountll(covered) != kFreeCount) {
    throw std::runtime_error("内部错误：路径没有覆盖全部可飞格");
  }
}

std::string routeText(const std::vector<int> & path)
{
  std::ostringstream result;
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (index != 0U) {
      result << ',';
    }
    result << cellCode(path[index]);
  }
  return result.str();
}
}  // namespace

class BestPathPlannerNode final : public rclcpp::Node {
public:
  BestPathPlannerNode()
  : Node("best_path_planner")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    serial_poll_ms_ = declare_parameter<int>("serial_poll_ms", 50);
    path_color_ = declare_parameter<int>("path_color", 2016);
    log_serial_rx_ = declare_parameter<bool>("log_serial_rx", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    cell_size_ = declare_parameter<double>("cell_size", 1.0);
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);

    if (serial_port_.empty()) {
      throw std::invalid_argument("参数 serial_port 不能为空");
    }
    if (serial_poll_ms_ < 10 || serial_poll_ms_ > 1000) {
      throw std::invalid_argument("参数 serial_poll_ms 必须在 10～1000 之间");
    }
    if (cell_size_ <= 0.0) {
      throw std::invalid_argument("参数 cell_size 必须大于 0");
    }
    if (path_color_ < 0 || path_color_ > 65535) {
      throw std::invalid_argument("参数 path_color 必须在 0～65535 之间");
    }
    (void)baudToTermios(serial_baud_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("patrol_path", qos);
    grid_path_publisher_ = create_publisher<std_msgs::msg::String>("patrol_path_grid", qos);
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(serial_poll_ms_), [this]() {onSerialTimer();});

    RCLCPP_INFO(
      get_logger(),
      "等待串口 %s（%d baud）接收3帧禁飞区数据，协议: 65 00 Ax By FF FF FF",
      serial_port_.c_str(), serial_baud_);
  }

  ~BestPathPlannerNode() override
  {
    closeSerial();
  }

private:
  bool openSerial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      return false;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "读取串口配置失败: %s", std::strerror(errno));
      closeSerial();
      return false;
    }
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    const speed_t speed = baudToTermios(serial_baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "设置串口配置失败: %s", std::strerror(errno));
      closeSerial();
      return false;
    }
    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(get_logger(), "串口已连接: %s", serial_port_.c_str());
    return true;
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
    serial_buffer_.clear();
  }

  void onSerialTimer()
  {
    if (serial_fd_ < 0) {
      const int retry_ticks = std::max(1, 1000 / serial_poll_ms_);
      if (reconnect_tick_++ % retry_ticks == 0 && !openSerial()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "无法打开串口 %s: %s，继续等待",
          serial_port_.c_str(), std::strerror(errno));
      }
      return;
    }

    std::uint8_t buffer[256];
    while (true) {
      const ssize_t count = ::read(serial_fd_, buffer, sizeof(buffer));
      if (count > 0) {
        if (log_serial_rx_) {
          RCLCPP_INFO(
            get_logger(), "串口收到 %zd 字节: %s", count,
            bytesToHex(buffer, static_cast<std::size_t>(count)).c_str());
        }
        serial_buffer_.insert(serial_buffer_.end(), buffer, buffer + count);
        if (serial_buffer_.size() > 4096U) {
          RCLCPP_ERROR(get_logger(), "串口缓存超过4096字节，已清空并重新同步");
          serial_buffer_.clear();
        }
        processSerialFrames();
        continue;
      }
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "读取串口失败: %s，将重新连接", std::strerror(errno));
        closeSerial();
      }
      break;
    }
    flushNextDrawCommand();
  }

  void processSerialFrames()
  {
    constexpr std::size_t frame_size = 7;
    while (true) {
      const auto header = std::find(serial_buffer_.begin(), serial_buffer_.end(), 0x65U);
      if (header == serial_buffer_.end()) {
        serial_buffer_.clear();
        return;
      }
      serial_buffer_.erase(serial_buffer_.begin(), header);
      if (serial_buffer_.size() < frame_size) {
        return;
      }

      const bool valid_frame =
        serial_buffer_[0] == 0x65U && serial_buffer_[1] == 0x00U &&
        serial_buffer_[4] == 0xFFU && serial_buffer_[5] == 0xFFU &&
        serial_buffer_[6] == 0xFFU;
      if (!valid_frame) {
        RCLCPP_WARN(
          get_logger(), "收到格式错误的串口帧，候选数据: %s；丢弃一个字节并重新同步",
          bytesToHex(serial_buffer_.data(), frame_size).c_str());
        serial_buffer_.erase(serial_buffer_.begin());
        continue;
      }

      const std::uint8_t a_code = serial_buffer_[2];
      const std::uint8_t b_code = serial_buffer_[3];
      serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + frame_size);
      processBlockedCode(a_code, b_code);
    }
  }

  void processBlockedCode(std::uint8_t a_code, std::uint8_t b_code)
  {
    int id = -1;
    try {
      id = serialBytesToCellId(a_code, b_code);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "无效禁飞区代码 0x%02X 0x%02X: %s",
        static_cast<unsigned int>(a_code), static_cast<unsigned int>(b_code), error.what());
      return;
    }

    if (std::find(received_blocked_.begin(), received_blocked_.end(), id) !=
      received_blocked_.end())
    {
      RCLCPP_WARN(
        get_logger(), "重复禁飞格 0x%02X 0x%02X -> %s，本帧不计数",
        static_cast<unsigned int>(a_code), static_cast<unsigned int>(b_code),
        cellCode(id).c_str());
      return;
    }

    received_blocked_.push_back(id);
    RCLCPP_INFO(
      get_logger(), "收到禁飞格 %zu/3: 0x%02X 0x%02X -> %s",
      received_blocked_.size(), static_cast<unsigned int>(a_code),
      static_cast<unsigned int>(b_code), cellCode(id).c_str());
    if (received_blocked_.size() < kBlockedCount) {
      return;
    }

    std::vector<std::string> codes;
    codes.reserve(kBlockedCount);
    for (const int blocked_id : received_blocked_) {
      codes.push_back(cellCode(blocked_id));
    }
    received_blocked_.clear();

    try {
      planAndPublish(codes);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "3个禁飞格无效，已丢弃本批次并等待下一组: %s", error.what());
    }
  }

  void planAndPublish(const std::vector<std::string> & codes)
  {
    const std::uint64_t blocked = validateBlocked(codes);
    const std::vector<int> path = planLowRepeat(blocked);
    checkPath(path, blocked);

    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.poses.reserve(path.size());
    for (const int id : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = origin_x_ + static_cast<double>(cellX(id) - 1) * cell_size_;
      pose.pose.position.y = origin_y_ + static_cast<double>(cellY(id) - 1) * cell_size_;
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    path_publisher_->publish(message);

    std_msgs::msg::String grid_message;
    grid_message.data = routeText(path);
    grid_path_publisher_->publish(grid_message);

    queueDrawCommands(path);
    RCLCPP_INFO(
      get_logger(), "OK COUNT=%zu PATH=%s", path.size(), grid_message.data.c_str());
  }

  void queueDrawCommands(const std::vector<int> & path)
  {
    draw_commands_.clear();
    transmit_offset_ = 0;
    if (path.size() < 2U) {
      return;
    }

    for (std::size_t index = 1; index < path.size(); ++index) {
      std::ostringstream text;
      text << "line "
           << screenX(path[index - 1U]) << ',' << screenY(path[index - 1U]) << ','
           << screenX(path[index]) << ',' << screenY(path[index]) << ',' << path_color_;
      const std::string command_text = text.str();
      std::vector<std::uint8_t> command(command_text.begin(), command_text.end());
      command.push_back(0xFFU);
      command.push_back(0xFFU);
      command.push_back(0xFFU);
      draw_commands_.push_back(std::move(command));
    }
    RCLCPP_INFO(
      get_logger(), "已生成 %zu 条串口屏画线指令，颜色=%d",
      draw_commands_.size(), path_color_);
  }

  void flushNextDrawCommand()
  {
    if (serial_fd_ < 0 || draw_commands_.empty()) {
      return;
    }

    const std::vector<std::uint8_t> & command = draw_commands_.front();
    const std::uint8_t * data = command.data() + transmit_offset_;
    const std::size_t remaining = command.size() - transmit_offset_;
    const ssize_t count = ::write(serial_fd_, data, remaining);
    if (count > 0) {
      transmit_offset_ += static_cast<std::size_t>(count);
      if (transmit_offset_ == command.size()) {
        draw_commands_.pop_front();
        transmit_offset_ = 0;
        if (draw_commands_.empty()) {
          RCLCPP_INFO(get_logger(), "全部路径画线指令已发送完成");
        }
      }
      return;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return;
    }
    if (count < 0) {
      RCLCPP_ERROR(get_logger(), "发送画线指令失败: %s，将重新连接", std::strerror(errno));
      closeSerial();
    }
  }

  std::vector<std::uint8_t> serial_buffer_;
  std::vector<int> received_blocked_;
  std::deque<std::vector<std::uint8_t>> draw_commands_;
  std::string serial_port_;
  std::string frame_id_;
  int serial_baud_{};
  int serial_poll_ms_{};
  int path_color_{};
  bool log_serial_rx_{true};
  int serial_fd_{-1};
  int reconnect_tick_{};
  std::size_t transmit_offset_{};
  double cell_size_{};
  double origin_x_{};
  double origin_y_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grid_path_publisher_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BestPathPlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("best_path_planner"), "节点启动失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
