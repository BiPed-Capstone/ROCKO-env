#ifndef ROCKO_ENV_SIM_PLANT
#define ROCKO_ENV_SIM_PLANT

#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "rocko_env/SimPlantModel.hpp"

namespace rocko_env
{
// The simulated robot, as a ros2_control hardware plugin (Tier 3, docs/architecture/sim-plant.md).
// It stands in for BOTH motor plugins and the IMU plugin at once and exports exactly the same 14
// interfaces they do, so the real controllers load against it unchanged:
//   left_wheel_joint, right_wheel_joint: command velocity [rad/s]; states position [m],
//   velocity [m/s], yaw [deg], pitch [deg], roll [deg], pitch_rate [deg/s].
// The physics is in SimPlantModel.hpp. Every physical number is an ESTIMATE, NOT MEASURED.
//
// "Released": the simulated robot is held still at its starting lean until the controllers have
// written a finite command to both wheels, like a person holding it until the controllers are
// running. The physics clock (and every scenario time) starts at that moment.
// Why the hold: controller_manager activates the hardware several seconds BEFORE the controllers
// are spawned (spawning is slow). A plant that began falling at on_activate would already be
// lying on the floor when the cascade started, and a scenario "2 seconds in" would mean nothing.
// Spec: docs/architecture/sim-plant.md §3.6, including the gap between release and the loops
// having a reference.
class SimPlant : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SimPlant);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  sim::SimPlantParams params_;              // filled from the xacro's <param> tags in on_init
  std::unique_ptr<sim::SimPlantModel> model_;  // created in on_init: the only allocation
  sim::SimImuOutput imu_;                   // the C-2 rule for a failed IMU read

  // The values the interfaces point at. Commands start NaN = "no controller has written yet".
  double left_command_rad_s_ = kNaN;
  double right_command_rad_s_ = kNaN;
  double position_m_ = 0.0;         // [m] both joints, as on the robot (left encoder on both)
  double left_velocity_m_s_ = 0.0;  // [m/s]
  double right_velocity_m_s_ = 0.0;
  double pitch_deg_ = kNaN;         // [deg] both joints, as on the robot (one IMU)
  double pitch_rate_deg_s_ = kNaN;  // [deg/s] both joints; the CURRENT rate, no IMU delay (H-6)
  double yaw_deg_ = 0.0;            // [deg] always 0: the planar model has no yaw
  double roll_deg_ = 0.0;           // [deg] always 0: the planar model has no roll

  double unintegrated_s_ = 0.0;  // [s] control time not yet turned into 1 ms physics steps (in [-1 ns, 1 ms))
  bool released_ = false;  // false: physics frozen (the hold above); set in write()
  bool fallen_reported_ = false;
  unsigned long clamped_reads_ = 0;  // read()s whose period needed more than kMaxSubStepsPerRead
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_SIM_PLANT
