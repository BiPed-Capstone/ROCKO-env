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
#include "rocko_env/PID4991.hpp"

namespace rocko_env
{
hardware_interface::CallbackReturn PID4991::on_init(
    const hardware_interface::HardwareInfo & info)
{
    _prefix = info.sensors[0].name;

    // Set up connection to client
    _node = rclcpp::Node::make_shared("pid4991_client");
    std::string s = _prefix + "_data";
    _client = _node->create_client<rocko_interfaces::srv::Pid4991Data>(s);

    while (!_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the %s service. Exiting.", s.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service not available, waiting again...", s.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PID4991::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto request = std::make_shared<rocko_interfaces::srv::Pid4991Data::Request>();

    auto result = _client->async_send_request(request);
    // Wait for the result.
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      // TODO: Grab the response data and put into variables for you to use
      _position = res->position;
      _velocity = res->velocity;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pos: %6.2f Vel: %5.2f", _position, _velocity);
      
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service PID4991");
    }

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> PID4991::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, POSITION_KEY, &_position));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, VELOCITY_KEY, &_velocity));

  return state_interfaces;
}


}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::PID4991, hardware_interface::SensorInterface)
