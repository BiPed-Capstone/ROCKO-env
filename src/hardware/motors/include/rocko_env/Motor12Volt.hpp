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

namespace rocko_env
{
class Motor12Volt : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Motor12Volt);

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
  std::string PIN_NAME_SPEED_KEY = "pinNameSpeed";
  std::string PIN_NAME_DIRECTION_KEY = "pinNameDirection";

  Wheel _wheel;
  std::string _pinNameSpeed;
  std::string _pinNameDirection;
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_MOTOR_12_VOLT