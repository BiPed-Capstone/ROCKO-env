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
    const hardware_interface::HardwareInfo & info)
{
    _prefix = info.sensors[0].name;

    // Set up connection to client
    _node = rclcpp::Node::make_shared("icm20948_client");
    _client = _node->create_client<rocko_interfaces::srv::Icm20948Data>("icm20948_data");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ICM20948::export_state_interfaces()
{
    // Export the member level vars we saved data to in read()
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, X_STATE_KEY, &_x));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, Y_STATE_KEY, &_y));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _prefix, Z_STATE_KEY, &_z));

  return state_interfaces;
}

hardware_interface::return_type ICM20948::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Ask gyro for its data

    auto request = std::make_shared<rocko_interfaces::srv::Icm20948Data::Request>();

    while (!_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = _client->async_send_request(request);
    // Wait for the result.
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      _x = res->x;
      _y = res->y;
      _z = res->z;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %3.2f y: %3.2f z: %3.2f\n", _x, _y, _z);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service icm20948_data");
    }

    return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::ICM20948, hardware_interface::SensorInterface)