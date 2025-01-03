// TODO: Change <CLASS_NAME> to name of your hardware class (use ctrl + f to find all)
#ifndef ROCKO_ENV_<CLASS_NAME>
#define ROCKO_ENV_<CLASS_NAME>

#include <memory>
#include <string>
#include <vector>  

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace rocko_env
{
class <CLASS_NAME> : public hardware_interface::SensorInterface // TODO: Change this to ActuatorInterface if you're implementing a motor
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(<CLASS_NAME>);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

// TODO: Uncomment these lines if you're implementing a motor

//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   hardware_interface::CallbackReturn on_activate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::CallbackReturn on_deactivate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::return_type write(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_<CLASS_NAME>