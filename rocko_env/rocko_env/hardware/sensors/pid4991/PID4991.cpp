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
#include "rocko_env/quad_encoder.hpp"

namespace rocko_env
{
hardware_interface::CallbackReturn PID4991::on_init(
  const hardware_interface::HardwareInfo & info)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PID4991::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

hardware_interface::return_type PID4991::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::PID4991, hardware_interface::SensorInterface)
