#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rocko_env/ICM20948.hpp"

namespace rocko_env
{

hardware_interface::CallbackReturn ICM20948::on_init(
    const hardware_interface::HardwareInfo & )
{
    // Handle any setup, which seems to be none since we don't actually have the hardware here

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ICM20948::export_state_interfaces()
{
    // Export the member level vars we saved data to in read()
  std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.vel));

//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     _wheel.name, hardware_interface::HW_IF_POSITION, &_wheel.pos));

  return state_interfaces;
}

hardware_interface::return_type ICM20948::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Ask gyro for its data

    return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::ICM20948, hardware_interface::SensorInterface)