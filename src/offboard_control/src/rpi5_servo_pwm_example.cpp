#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace fs = std::filesystem;

namespace
{
constexpr int kPeriodNs = 20'000'000;      // 50 Hz, standard servo PWM period.
constexpr int kMinPulseUs = 500;           // Servo 0 degree pulse.
constexpr int kMaxPulseUs = 2500;          // Servo 180 degree pulse.
constexpr double kMaxServoAngleDeg = 180.0;

void write_file(const fs::path & path, const std::string & value)
{
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open " + path.string());
  }

  file << value;
  if (!file) {
    throw std::runtime_error("failed to write " + path.string());
  }
}

void export_pwm_if_needed(const fs::path & chip_path, int channel)
{
  const fs::path pwm_path = chip_path / ("pwm" + std::to_string(channel));
  if (fs::exists(pwm_path)) {
    return;
  }

  write_file(chip_path / "export", std::to_string(channel));

  for (int i = 0; i < 50; ++i) {
    if (fs::exists(pwm_path)) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  throw std::runtime_error("PWM channel did not appear after export: " + pwm_path.string());
}

int angle_to_duty_ns(double angle_deg)
{
  angle_deg = std::clamp(angle_deg, 0.0, kMaxServoAngleDeg);
  const double pulse_us =
    kMinPulseUs + (angle_deg / kMaxServoAngleDeg) * (kMaxPulseUs - kMinPulseUs);
  return static_cast<int>(std::lround(pulse_us * 1000.0));
}

void set_servo_angle(const fs::path & pwm_path, double angle_deg)
{
  write_file(pwm_path / "duty_cycle", std::to_string(angle_to_duty_ns(angle_deg)));
}

void print_pwm_value(double angle_deg)
{
  const int duty_ns = angle_to_duty_ns(angle_deg);
  std::cout
    << angle_deg << " deg -> period " << kPeriodNs << " ns, duty "
    << duty_ns << " ns (" << duty_ns / 1000.0 << " us)\n";
}

void print_usage(const char * program)
{
  std::cerr
    << "Usage: " << program << " [pwmchip] [channel]\n"
    << "Example: sudo " << program << " 0 0\n\n"
    << "Before running on Raspberry Pi 5B:\n"
    << "  1. Enable a hardware PWM overlay for your chosen GPIO pin.\n"
    << "  2. Reboot, then check /sys/class/pwm/pwmchip*/.\n"
    << "  3. Connect servo signal to the PWM GPIO, servo GND to Pi GND,\n"
    << "     and use a suitable external 5V supply for the servo.\n";
}
}  // namespace

int main(int argc, char * argv[])
{
  int chip = 0;
  int channel = 0;

  try {
    if (argc > 1) {
      chip = std::stoi(argv[1]);
    }
    if (argc > 2) {
      channel = std::stoi(argv[2]);
    }
    if (argc > 3 || chip < 0 || channel < 0) {
      print_usage(argv[0]);
      return 2;
    }

    const fs::path chip_path = fs::path("/sys/class/pwm") / ("pwmchip" + std::to_string(chip));
    if (!fs::exists(chip_path)) {
      throw std::runtime_error(
        chip_path.string() + " does not exist; enable PWM overlay or choose another pwmchip");
    }

    export_pwm_if_needed(chip_path, channel);
    const fs::path pwm_path = chip_path / ("pwm" + std::to_string(channel));

    write_file(pwm_path / "enable", "0");
    write_file(pwm_path / "period", std::to_string(kPeriodNs));
    if (fs::exists(pwm_path / "polarity")) {
      write_file(pwm_path / "polarity", "normal");
    }

    set_servo_angle(pwm_path, 0.0);
    print_pwm_value(0.0);
    write_file(pwm_path / "enable", "1");
    std::cout << "Servo at 0 degree\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int angle = 1; angle <= 45; ++angle) {
      set_servo_angle(pwm_path, static_cast<double>(angle));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    print_pwm_value(45.0);
    std::cout << "Servo moved to 45 degrees\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    write_file(pwm_path / "enable", "0");
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "Error: " << ex.what() << "\n\n";
    print_usage(argv[0]);
    return 1;
  }
}
