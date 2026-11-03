// SimPlant: ROCKO's physics simulation as a ros2_control hardware plugin (Tier 3).
// Specification: docs/architecture/sim-plant.md. Physics: include/rocko_env/SimPlantModel.hpp.
// Every physical parameter is an ESTIMATE, NOT MEASURED; gains that work here are candidates only.

#include "rocko_env/SimPlant.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rocko_env
{

namespace
{
// A period within this many sub-steps of a whole number counts as that whole number (floating-point
// rounding in "0.01 s / 0.001 s" and in the carried remainder).
constexpr double kStepSlack = 1e-6;  // [sub-steps]

// Minimum time between repeats of the same throttled log line in this file.
constexpr int kLogThrottleMs = 1000;  // [ms]

// Interface names. The same strings the real plugins (Motor12VoltQuadEncoder, ICM20948) and the
// MockRocko block use, in the same order as the xacro lists them.
const char * const kJointNames[] = {"left_wheel_joint", "right_wheel_joint"};
const char * const kStateNames[] = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, "yaw", "pitch", "roll",
  "pitch_rate"};

// "true" / "false" only. hardware_interface::parse_bool reads anything else as false, which would
// turn a typo like disable_torque=ture into a silent false.
bool parse_strict_bool(const std::string & text, bool & out)
{
  if (text == "true") {out = true; return true;}
  if (text == "false") {out = false; return true;}
  return false;
}

// A whole integer or nothing: std::stoi alone would read "20ms" as 20. Throws std::exception
// (std::invalid_argument or std::out_of_range) when the text is not exactly one integer.
int parse_strict_int(const std::string & text)
{
  std::size_t used = 0;
  const int value = std::stoi(text, &used);
  if (used != text.size()) {
    throw std::invalid_argument("trailing characters after the number in '" + text + "'");
  }
  return value;
}

// The joints and interfaces must be exactly the real hardware's (spec §3.7), or the real controllers
// would not load against this plugin. Returns false (after logging why) if they are not.
bool check_joint_interfaces(
  const hardware_interface::HardwareInfo & info, const rclcpp::Logger & logger)
{
  const std::size_t joint_count = std::size(kJointNames);
  const std::size_t state_count = std::size(kStateNames);
  if (info.joints.size() != joint_count) {
    RCLCPP_FATAL(logger, "SimPlant needs %zu joints, got %zu.", joint_count, info.joints.size());
    return false;
  }
  for (std::size_t j = 0; j < joint_count; ++j) {
    const auto & joint = info.joints[j];
    bool ok = joint.name == kJointNames[j] && joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY &&
      joint.state_interfaces.size() == state_count;
    for (std::size_t s = 0; ok && s < state_count; ++s) {
      ok = joint.state_interfaces[s].name == kStateNames[s];
    }
    if (!ok) {
      RCLCPP_FATAL(
        logger, "Joint %zu ('%s') must be '%s' with command 'velocity' and states position, "
        "velocity, yaw, pitch, roll, pitch_rate in that order (the real hardware's interfaces).",
        j, joint.name.c_str(), kJointNames[j]);
      return false;
    }
  }
  return true;
}

// Fills `params` from the xacro's <param> tags. Every parameter must be given (the xacro gives
// them all), so the value in use is always the one in the xacro; a name the plugin does not know is
// refused, so a typo cannot silently leave a default in place. Returns false (after logging why)
// if anything is missing, unknown, malformed or physically impossible.
//
// ADDING A PARAMETER means touching four places: sim::SimPlantParams (SimPlantModel.hpp), the arg in
// rocko.urdf.xacro, the <param> in rocko.ros2_control.xacro, and the tables below.
// sim/check_sim_config.py (run by verify_urdf.sh) checks the first three agree; a table entry that
// is missing or misspelled is caught by this function the first time the plugin loads.
bool read_parameters(
  const hardware_interface::HardwareInfo & info, sim::SimPlantParams & params,
  const rclcpp::Logger & logger)
{
  struct DoubleParam {const char * name; double * value;};
  const DoubleParam doubles[] = {
    {"wheel_radius_m", &params.wheel_radius_m},
    {"belt_ratio", &params.belt_ratio},
    {"wheel_mass_kg", &params.wheel_mass_kg},
    {"body_mass_kg", &params.body_mass_kg},
    {"com_height_m", &params.com_height_m},
    {"body_height_m", &params.body_height_m},
    {"body_depth_m", &params.body_depth_m},
    {"gravity_m_s2", &params.gravity_m_s2},
    {"motor_stall_torque_Nm", &params.motor_stall_torque_Nm},
    {"motor_no_load_speed_rad_s", &params.motor_no_load_speed_rad_s},
    {"motor_stall_current_A", &params.motor_stall_current_A},
    {"motor_no_load_current_A", &params.motor_no_load_current_A},
    {"coulomb_friction_Nm", &params.coulomb_friction_Nm},
    {"battery_voltage_V", &params.battery_voltage_V},
    {"motor_rated_voltage_V", &params.motor_rated_voltage_V},
    {"imu_pitch_bias_deg", &params.imu_pitch_bias_deg},
    {"fallen_pitch_deg", &params.fallen_pitch_deg},
    {"pitch_sign", &params.pitch_sign},
    {"encoder_sign", &params.encoder_sign},
    {"initial_pitch_deg", &params.initial_pitch_deg},
    {"imu_fault_start_s", &params.imu_fault_start_s},
    {"imu_fault_duration_s", &params.imu_fault_duration_s},
    {"impulse_time_s", &params.impulse_time_s},
    {"impulse_angular_velocity_deg_per_s", &params.impulse_angular_velocity_deg_per_s},
  };
  struct IntParam {const char * name; int * value;};
  const IntParam ints[] = {
    {"imu_delay_ms", &params.imu_delay_ms},
    {"imu_delay_max_ms", &params.imu_delay_max_ms},
  };
  struct BoolParam {const char * name; bool * value;};
  const BoolParam bools[] = {
    {"model_write_quirks", &params.model_write_quirks},
    {"model_encoder_node", &params.model_encoder_node},
    {"disable_torque", &params.disable_torque},
  };

  const auto & given = info.hardware_parameters;
  std::set<std::string> known;
  auto is_missing = [&](const char * name) {
      known.insert(name);
      if (given.count(name)) {return false;}
      RCLCPP_FATAL(logger, "SimPlant parameter '%s' is missing from the xacro.", name);
      return true;
    };
  for (const auto & p : doubles) {
    if (is_missing(p.name)) {return false;}
    try {
      *p.value = hardware_interface::stod(given.at(p.name));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(logger, "SimPlant parameter '%s' must be a number, got '%s' (%s).", p.name,
        given.at(p.name).c_str(), e.what());
      return false;
    }
  }
  for (const auto & p : ints) {
    if (is_missing(p.name)) {return false;}
    try {
      *p.value = parse_strict_int(given.at(p.name));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(logger, "SimPlant parameter '%s' must be a whole number, got '%s' (%s).", p.name,
        given.at(p.name).c_str(), e.what());
      return false;
    }
  }
  for (const auto & p : bools) {
    if (is_missing(p.name)) {return false;}
    if (!parse_strict_bool(given.at(p.name), *p.value)) {
      RCLCPP_FATAL(logger, "SimPlant parameter '%s' must be true or false, got '%s'.", p.name,
        given.at(p.name).c_str());
      return false;
    }
  }
  for (const auto & [name, value] : given) {
    if (!known.count(name)) {
      RCLCPP_FATAL(logger, "SimPlant has no parameter '%s' (a typo?).", name.c_str());
      return false;
    }
  }
  if (const char * why = sim::invalid_reason(params)) {
    RCLCPP_FATAL(logger, "SimPlant parameters rejected: %s.", why);
    return false;
  }
  return true;
}
}  // namespace

hardware_interface::CallbackReturn SimPlant::on_init(const hardware_interface::HardwareInfo & info)
{
  if (!check_joint_interfaces(info, get_logger()) ||
    !read_parameters(info, params_, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  model_ = std::make_unique<sim::SimPlantModel>(params_);
  RCLCPP_WARN(
    get_logger(), "SIMULATED ROBOT (SimPlant): a model with ESTIMATED, unmeasured parameters. "
    "Nothing here says how the real robot behaves. pitch_sign=%+.0f encoder_sign=%+.0f "
    "imu_delay_ms=%d initial_pitch_deg=%.2f disable_torque=%s", params_.pitch_sign,
    params_.encoder_sign, params_.imu_delay_ms, params_.initial_pitch_deg,
    params_.disable_torque ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SimPlant::export_state_interfaces()
{
  // Order and names as in the xacro. One position and one pitch value feed both joints, as on the
  // robot (the encoder node reports the left wheel to both; there is one IMU).
  std::vector<hardware_interface::StateInterface> states;
  double * velocity[] = {&left_velocity_m_s_, &right_velocity_m_s_};
  for (std::size_t j = 0; j < std::size(kJointNames); ++j) {
    states.emplace_back(kJointNames[j], hardware_interface::HW_IF_POSITION, &position_m_);
    states.emplace_back(kJointNames[j], hardware_interface::HW_IF_VELOCITY, velocity[j]);
    states.emplace_back(kJointNames[j], "yaw", &yaw_deg_);
    states.emplace_back(kJointNames[j], "pitch", &pitch_deg_);
    states.emplace_back(kJointNames[j], "roll", &roll_deg_);
    states.emplace_back(kJointNames[j], "pitch_rate", &pitch_rate_deg_s_);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> SimPlant::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  commands.emplace_back(kJointNames[0], hardware_interface::HW_IF_VELOCITY, &left_command_rad_s_);
  commands.emplace_back(kJointNames[1], hardware_interface::HW_IF_VELOCITY, &right_command_rad_s_);
  return commands;
}

hardware_interface::CallbackReturn SimPlant::on_activate(const rclcpp_lifecycle::State &)
{
  // A fresh start of the scenario (spec §3.6). Nothing here allocates.
  model_->reset();
  imu_.reset();
  left_command_rad_s_ = kNaN;
  right_command_rad_s_ = kNaN;
  pitch_deg_ = imu_.pitch_deg();  // NaN until the first read
  pitch_rate_deg_s_ = imu_.pitch_rate_deg_s();  // NaN until the first read
  released_ = false;
  unintegrated_s_ = 0.0;
  fallen_reported_ = false;
  clamped_reads_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimPlant::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(), "SimPlant stopped at sim time %.3f s after release; %lu read()s were clamped to "
    "%d ms of physics; %lu failed IMU reads.", model_->time_s(), clamped_reads_,
    sim::kMaxSubStepsPerRead, imu_.tracker().total_failures());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SimPlant::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Advance the physics by this cycle's period, in 1 ms sub-steps, so the physics clock and the
  // controllers' dt agree. Nothing moves before release (see SimPlant.hpp). A period that is not a
  // whole number of sub-steps leaves a remainder, carried to the next read() so no time is lost.
  // More than kMaxSubStepsPerRead means the container stalled; that much is skipped (and counted)
  // rather than integrated in one burst.
  if (released_) {
    // A negative period (a clock step backwards) would become time the physics owes later and
    // freeze the plant while the controllers keep running, so it counts as zero.
    unintegrated_s_ += std::max(period.seconds(), 0.0);
    // kStepSlack: a remainder like 0.9999999999 sub-steps from rounding error counts as a whole one.
    const long wanted = static_cast<long>(std::floor(unintegrated_s_ / sim::kSubStepS + kStepSlack));
    const long steps = std::clamp(wanted, 0L, static_cast<long>(sim::kMaxSubStepsPerRead));
    if (wanted > sim::kMaxSubStepsPerRead) {
      ++clamped_reads_;
      unintegrated_s_ = 0.0;  // the skipped time is gone, not owed
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
        "SimPlant: a %.1f ms control period was cut to %d ms of physics (%lu times so far).",
        period.seconds() * 1000.0, sim::kMaxSubStepsPerRead, clamped_reads_);
    } else {
      unintegrated_s_ -= static_cast<double>(steps) * sim::kSubStepS;
    }
    for (long i = 0; i < steps; ++i) {
      model_->step();
    }
  } else {
    unintegrated_s_ = 0.0;
  }

  model_->update_encoder_outputs();
  position_m_ = model_->position_m();
  left_velocity_m_s_ = model_->left_velocity_m_s();
  right_velocity_m_s_ = model_->right_velocity_m_s();

  if (model_->fallen() && !fallen_reported_) {
    fallen_reported_ = true;  // logged once, not per cycle
    RCLCPP_WARN(get_logger(), "SimPlant: the simulated body passed %.0f deg at sim time %.3f s and "
      "is now lying on the floor (lean frozen).", params_.fallen_pitch_deg, model_->time_s());
  }

  // The IMU (spec §5). An injected outage is a failed read: keep the last good pitch, count it,
  // and return ERROR once the fault tracker trips, exactly as ICM20948::read() does (finding C-2).
  const double t = model_->time_s();
  const bool imu_outage = released_ && params_.imu_fault_duration_s > 0.0 &&
    t >= params_.imu_fault_start_s && t < params_.imu_fault_start_s + params_.imu_fault_duration_s;
  // update() returns false once the fault tracker has tripped, i.e. read() must now return ERROR
  // (a good read always gives true). On a failed read it keeps the last good pitch, never 0.0.
  // A NaN from the model is a failed read too (never exported as a pitch). The pitch rate (finding
  // H-6) comes from the same simulated read: both good, or the whole read failed.
  const double model_pitch_deg = model_->imu_pitch_deg();
  const double model_pitch_rate_deg_s = model_->imu_pitch_rate_deg_s();
  const bool imu_ok = imu_.update(
    !imu_outage && std::isfinite(model_pitch_deg) && std::isfinite(model_pitch_rate_deg_s),
    model_pitch_deg, model_pitch_rate_deg_s);
  pitch_deg_ = imu_.pitch_deg();
  pitch_rate_deg_s_ = imu_.pitch_rate_deg_s();
  if (imu_outage) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
      "SimPlant: injected IMU read failure: %d failed in a row, %lu in total. Holding the last "
      "good pitch and pitch rate.", imu_.tracker().consecutive_failures(), imu_.tracker().total_failures());
  }
  if (!imu_ok) {
    return hardware_interface::return_type::ERROR;
  }

  // A state that is not a number means the model blew up: say so rather than export it.
  if (!model_->state_is_finite() || !std::isfinite(position_m_) ||
    !std::isfinite(left_velocity_m_s_) || !std::isfinite(right_velocity_m_s_))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
      "SimPlant: the simulated state is not finite; returning ERROR.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimPlant::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Release (why: SimPlant.hpp). controller_manager activates this hardware seconds before the
  // controllers are spawned; without the hold the robot would be on the floor before the cascade
  // started. Once both wheels get a finite command, physics starts at the next read(). Before that
  // a command is NaN, which the write-path copy turns into 0 % duty anyway.
  if (!released_ && std::isfinite(left_command_rad_s_) && std::isfinite(right_command_rad_s_)) {
    released_ = true;  // logged once, not per cycle
    RCLCPP_INFO(get_logger(), "SimPlant: controllers are commanding both wheels; releasing the "
      "simulated robot (sim time starts now).");
  }
  // The duty each motor gets, through a copy of Motor12VoltQuadEncoder::write(). It acts from the
  // next read() until the following write(): a zero-order hold, like softPwm on the robot.
  model_->set_wheel_commands(left_command_rad_s_, right_command_rad_s_);
  return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rocko_env::SimPlant, hardware_interface::SystemInterface)
