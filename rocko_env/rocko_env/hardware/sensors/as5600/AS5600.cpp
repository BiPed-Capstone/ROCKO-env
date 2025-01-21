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

#include "rocko_env/AS5600.hpp"

namespace rocko_env
{

hardware_interface::CallbackReturn AS5600::on_init(
    const hardware_interface::HardwareInfo & info)
{
    _prefix = info.sensors[0].name;

    // Set up connection to client
    _node = rclcpp::Node::make_shared(_prefix + "_client");
    std::string s = _prefix + "_data";
    _client = _node->create_client<rocko_interfaces::srv::As5600Data>(s);
    
    while (!_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s service not available, waiting again...", _prefix.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AS5600::export_state_interfaces()
{
    // Export the member level vars we saved data to in read()
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, ANGLE_KEY, &_angle));

  return state_interfaces;
}

hardware_interface::return_type AS5600::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Ask gyro for its data

    auto request = std::make_shared<rocko_interfaces::srv::As5600Data::Request>();

    auto result = _client->async_send_request(request);
    // Wait for the result.
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      _angle = res->angle;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle %5.2f\n", _angle);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s_data", _prefix.c_str());
    }

    return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::AS5600, hardware_interface::SensorInterface)