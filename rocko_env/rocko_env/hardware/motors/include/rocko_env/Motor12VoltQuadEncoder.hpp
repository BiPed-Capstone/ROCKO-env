#ifndef ROCKO_ENV_MOTOR_12_VOLT
#define ROCKO_ENV_MOTOR_12_VOLT

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "wheel.hpp"

#include "rocko_interfaces/srv/quad_encoder_data.hpp"

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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void calcCurrentEncVal();

  std::string PIN_NUMBER_SPEED_KEY = "pinNumberSpeed";
  std::string PIN_NUMBER_DIRECTION_KEY = "pinNumberDirection";
  std::string INVERT_KEY = "invert";

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

  std::string _prefix = "";

  rclcpp::Client<rocko_interfaces::srv::QuadEncoderData>::SharedPtr _client;
  std::shared_ptr<rclcpp::Node> _node;
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_MOTOR_12_VOLT