#ifndef ROCKO_ENV_ICM20948
#define ROCKO_ENV_ICM20948

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "rocko_interfaces/srv/icm20948_data.hpp"

#include "rocko_env/ImuFaultTracker.hpp"

namespace rocko_env
{
class ICM20948 : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string _prefix = "";

  rclcpp::Client<rocko_interfaces::srv::Icm20948Data>::SharedPtr _client;
  std::shared_ptr<rclcpp::Node> _node;

  std::string YAW_KEY = "yaw";
  std::string PITCH_KEY = "pitch";
  std::string ROLL_KEY = "roll";

  // SAFETY PARAMETERS. Proposed values; a human confirms them on the robot. They are edit-and-
  // rebuild constants, not ROS parameters.
  //
  // Finding C-2 (docs/claude-analysis/balance-loop-findings.md): read() returns ERROR once this many
  // reads in a row have failed. Nominally 5 cycles x 10 ms = 50 ms at update_rate 100. It can be
  // longer in wall-clock time, because a failed read may block past 10 ms (finding C-1, the
  // blocking service call, is not fixed).
  static constexpr int kMaxConsecutiveImuFailures = 5;  // [reads] = [control cycles]
  // read() also returns ERROR when this many of the last ImuFaultTracker::kWindowReads reads
  // failed, which catches a sensor that alternates good and bad reads. Tracker default.
  static constexpr int kMaxWindowImuFailures = ImuFaultTracker::kMaxWindowFailures;  // [reads]
  ImuFaultTracker _imu_fault_tracker{kMaxConsecutiveImuFailures, kMaxWindowImuFailures};
  static_assert(
    kMaxConsecutiveImuFailures > 0 && kMaxWindowImuFailures > 0 &&
      kMaxWindowImuFailures <= ImuFaultTracker::kWindowReads,
    "fault limits must be positive, and the windowed limit must fit in the window");

  // A reply outside these limits is treated as a failed read; the BNO055 Euler output is normally
  // inside them. The reported pitch already includes the node's +7.6 deg pitch_offset, so a real
  // pitch in (172.4, 180] deg is also rejected. That only matters if the robot is upside down.
  static constexpr double kMaxAbsPitchRollDeg = 180.0;  // [deg] |pitch|, |roll| above this = bad
  static constexpr double kMaxAbsYawDeg = 360.0;        // [deg] |yaw| above this = bad

  // Angles in degrees as the BNO055 reports them (pitch includes the node's pitch_offset).
  // NaN until the first good read, so a failure on the very first cycle cannot look like 0.0.
  double _yaw = std::numeric_limits<double>::quiet_NaN();
  double _pitch = std::numeric_limits<double>::quiet_NaN();
  double _roll = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_ICM20948