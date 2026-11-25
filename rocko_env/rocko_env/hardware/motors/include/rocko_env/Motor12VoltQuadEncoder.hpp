#ifndef ROCKO_ENV_MOTOR_12_VOLT
#define ROCKO_ENV_MOTOR_12_VOLT

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "wheel.hpp"
#include "rocko_env/WheelEncoder.hpp"

struct WPIWfiStatus;  // WiringPi's interrupt record (wiringPi.h); only the .cpp needs its contents

namespace rocko_env
{
class Motor12VoltQuadEncoder : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Motor12VoltQuadEncoder);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // Stops the encoder interrupts: their callbacks hold a pointer to this object.
  ~Motor12VoltQuadEncoder() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Called by WiringPi on its interrupt thread for an edge on either encoder pin (see the .cpp).
  static void on_encoder_edge(struct WPIWfiStatus status, void * userdata);
  void stop_encoder_interrupts();
  void stop_motor();  // 0 % duty and end software PWM, if it was started

  std::string PIN_NUMBER_SPEED_KEY = "pinNumberSpeed";
  std::string PIN_NUMBER_DIRECTION_KEY = "pinNumberDirection";
  std::string INVERT_KEY = "invert";
  std::string ENCODER_PIN_A_KEY = "encoder_pin_a";
  std::string ENCODER_PIN_B_KEY = "encoder_pin_b";
  std::string WHEEL_RADIUS_KEY = "wheel_radius_m";
  std::string COUNTS_PER_REV_KEY = "encoder_counts_per_wheel_rev";

  double MAX_RAD_PER_SEC = 1150 / 60 * M_PI * 2; // Max RPM / 60 sec * 2pi

  // MOTOR OUTPUT PARAMETERS. Proposed values; a human confirms them on the robot. They are
  // edit-and-rebuild constants, not ROS parameters.
  // A command smaller than this is "stop": write() puts 0 % on the speed pin.
  static constexpr double kDeadbandRadPerSec = 0.05;  // [rad/s] wheel speed, either direction
  // Duty added on top of the scaled command so a small command still turns the motor (static
  // friction, "stiction"). The old code hid this as "+ 0.02" inside the percent calculation.
  static constexpr double kStictionDutyPercent = 2.0;  // [% of full duty], 0..100

  Wheel _wheel;
  int _speedPin;
  int _dirPin;
  bool _invert;
  bool _pwmRunning = false;  // softPwmCreate() done and not yet stopped

  // ---- The wheel encoder (finding C-1: counted here, no service call, no wait) ----
  int _encoderPinA = -1;            // WiringPi pin numbers (hardware parameters in the xacro)
  int _encoderPinB = -1;
  double _countsPerMetre = 0.0;     // [counts / m of wheel travel], from the radius and counts per turn
  bool _encoderInterruptsOn = false;

  // Written only by the two interrupt threads (one per pin), under _encoderMutex, so an edge on A and
  // one on B cannot step the decoder at the same time. read() never takes this mutex.
  std::mutex _encoderMutex;
  int _encoderState = 0;            // last quadrature state (0..3), see WheelEncoder.hpp
  // [counts] forward positive. Atomic so read() (the control thread) can load it without a lock while
  // an interrupt thread adds to it. Lock-free on the Pi (64-bit ARM), checked in the .cpp.
  std::atomic<std::int64_t> _encoderCount{0};

  WheelVelocityFilter _velocityFilter;  // used only by read(), on the control thread
  std::chrono::steady_clock::time_point _lastReadTime;  // when read() last took the count
  bool _haveLastReadTime = false;
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_MOTOR_12_VOLT