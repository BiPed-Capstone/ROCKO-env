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
    // Set up connection to client
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("icm20948_client");
    rclcpp::Client<rocko_interfaces::srv::Icm20948Data>::SharedPtr client = node->create_client<rocko_interfaces::srv::Icm20948Data>("icm20948");

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



    // auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    // request->a = atoll(argv[1]);
    // request->b = atoll(argv[2]);

    // while (!client->wait_for_service(1s)) {
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //     return 0;
    //   }
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    // }

    // auto result = client->async_send_request(request);
    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(node, result) ==
    //   rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    // } else {
    //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    // }

    return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::ICM20948, hardware_interface::SensorInterface)