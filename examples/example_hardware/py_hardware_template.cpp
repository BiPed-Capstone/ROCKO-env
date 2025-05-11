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

// TODO: include your .hpp file

// TODO: Replace <CLASS_NAME> with your class name

namespace rocko_env
{
hardware_interface::CallbackReturn <CLASS_NAME>::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Set up connection to client
  _node = rclcpp::Node::make_shared("<ClassName>_client");
  // TODO: Change ServiceType to the type of service in rocko_interfaces
  // TODO: Make the string in here match the service name in your .py file
  _client = _node->create_client<ServiceType>("<ClassName>_service");
  
  while (!_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> <CLASS_NAME>::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

hardware_interface::return_type <CLASS_NAME>::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Change ServiceType to the type of service you made in rocko_interfaces
  auto request = std::make_shared<ServiceType::Request>();

  // TODO: Fill in data for your request

  auto result = _client->async_send_request(request);
  // Wait for the result.
  if ((rclcpp::spin_until_future_complete(_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) && result.valid())
  {
    auto res = result.get();
    // TODO: Grab the response data and put into variables for you to use
    
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service <ClassName>");
  }

  return hardware_interface::return_type::OK;
}

// TODO: Uncomment if implementing a motor

// std::vector<hardware_interface::CommandInterface> <CLASS_NAME>::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   return command_interfaces;
// }

// hardware_interface::CallbackReturn <CLASS_NAME>::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn <CLASS_NAME>::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type <CLASS_NAME>::write(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
  // TODO: Change ServiceType to the type of service you made in rocko_interfaces
  // auto request = std::make_shared<ServiceType::Request>();

  // // TODO: Fill in data for your request

  // auto result = _client->async_send_request(request);
  // // Wait for the result.
  // if ((rclcpp::spin_until_future_complete(_node, result) ==
  //   rclcpp::FutureReturnCode::SUCCESS) && result.valid())
  // {
  //   auto res = result.get();
  //   // TODO: Grab the response data and put into variables for you to use
    
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service <ClassName>");
  // }

  // return hardware_interface::return_type::OK;
// }

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::<CLASS_NAME>, hardware_interface::SensorInterface) // TODO: Change this to ActuatorInterface if implementing a motor
