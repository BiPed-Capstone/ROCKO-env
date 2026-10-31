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

namespace
{
// Minimum time between repeats of the same throttled log line in this file.
constexpr int kLogThrottleMs = 1000;  // [ms]
}  // namespace

hardware_interface::CallbackReturn ICM20948::on_init(
    const hardware_interface::HardwareInfo & )
{
    // _prefix = info.sensors[0].name;

    // Set up connection to client
    _node = rclcpp::Node::make_shared("icm20948_client");
    _client = _node->create_client<rocko_interfaces::srv::Icm20948Data>("icm20948_data");

    while (!_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ICM20948::export_state_interfaces()
{
    // Export the member level vars we saved data to in read()
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", YAW_KEY, &_yaw));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", PITCH_KEY, &_pitch));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", ROLL_KEY, &_roll));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", YAW_KEY, &_yaw));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", PITCH_KEY, &_pitch));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", ROLL_KEY, &_roll));

  return state_interfaces;
}

// Reads yaw, pitch and roll [deg] from the Python node and stores them for the controllers.
//
// On a failed read the last good angles are kept: never 0.0 (NaN before the first good read),
// because pitch 0.0 is the balanced setpoint and a failure that looked like 0.0 would tell the
// pitch controller "level" while the robot is blind (finding C-2, see
// docs/claude-analysis/balance-loop-findings.md). Failures are counted; read() returns ERROR once
// _imu_fault_tracker trips. A single dropped sample returns OK on purpose.
hardware_interface::return_type ICM20948::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto request = std::make_shared<rocko_interfaces::srv::Icm20948Data::Request>();
    auto result = _client->async_send_request(request);

    // Blocks until the Python node replies; that can take longer than the 10 ms cycle
    // (finding C-1, not fixed here).
    bool good_read = false;
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      // The node sets success=false (and NaN angles) when the sensor read failed. Check the angles
      // too, so a bad or garbled sample is never stored even if `success` were wrong.
      const bool angles_ok =
        std::isfinite(res->yaw) && std::isfinite(res->pitch) && std::isfinite(res->roll) &&
        std::fabs(res->pitch) <= kMaxAbsPitchRollDeg &&
        std::fabs(res->roll) <= kMaxAbsPitchRollDeg &&
        std::fabs(res->yaw) <= kMaxAbsYawDeg;
      if (res->success && angles_ok) {
        _yaw = res->yaw;
        _pitch = res->pitch;
        _roll = res->roll;
        good_read = true;
      }
    }

    _imu_fault_tracker.record(good_read);
    if (good_read) {
      return hardware_interface::return_type::OK;
    }

    RCLCPP_ERROR_THROTTLE(
      _node->get_logger(), *_node->get_clock(), kLogThrottleMs,
      "IMU read failed: %d failed in a row, %d of the last %d reads, %lu in total. "
      "Holding the last good angles.",
      _imu_fault_tracker.consecutive_failures(), _imu_fault_tracker.window_failures(),
      ImuFaultTracker::kWindowReads, _imu_fault_tracker.total_failures());

    if (_imu_fault_tracker.tripped()) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::ICM20948, hardware_interface::SensorInterface)