#ifndef ROCKO_ENV_SIM_PLANT_MODEL
#define ROCKO_ENV_SIM_PLANT_MODEL

// The physics of the simulated ROCKO: a planar inverted pendulum on one wheel pair, driven by a
// DC-motor model. Plain C++17 and the standard library only, no ROS types, so a unit test on a
// laptop can call it directly (tools/sim_plant_model_test.cpp). The ros2_control plugin that wraps
// it is SimPlant.cpp.
//
// THE SPECIFICATION is docs/architecture/sim-plant.md. Section numbers below (§3.2, ...) refer to
// it. Read its "Read this first" box: every physical number here is an ESTIMATE, NOT MEASURED, and
// a gain that holds this model up says nothing about the real robot.
//
// Words (spec §1):
//   sim-forward  the direction a POSITIVE wheel command drives the robot. Whether that is the
//                robot's front or back is unknown (spec §2.4); the physics does not care.
//   psi (lean)   body tilt from vertical [rad], + = leaning toward sim-forward.
//   phi          wheel angle relative to the FLOOR [rad], + = rolling toward sim-forward.
//   phi - psi    wheel angle relative to the BODY: what the motor shaft and the encoder turn through.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "rocko_env/ImuFaultTracker.hpp"

namespace rocko_env
{
namespace sim
{

constexpr double kPi = 3.14159265358979323846;
inline double deg_to_rad(double deg) { return deg * kPi / 180.0; }
inline double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

// ---- Copied from the real code. NOT estimates: change them only when that code changes. -------
// Motor12VoltQuadEncoder.hpp writes MAX_RAD_PER_SEC = 1150 / 60 * M_PI * 2. In C++ `1150 / 60` is
// INTEGER division (= 19), so the constant is 19 * 2 pi = 119.381 rad/s, not 1150 RPM = 120.428
// rad/s (finding U-3). The bug is copied on purpose, so the sim maps commands to duty exactly as the
// robot does. Do not "fix" it here; fix it in the motor plugin and then here, in one commit.
constexpr double kWriteMaxRadPerSec = (1150 / 60) * 2.0 * kPi;  // [rad/s] = 119.381
// Do not "tidy" this into 1150.0 / 60.0: the motor model's no-load speed (motor_no_load_speed_rad_s
// below) is the OTHER spelling, in floating point on purpose, because it describes the motor and
// not the code. The two differ by 0.9 %; the assert stops the integer division being repaired by
// accident.
static_assert(
  kWriteMaxRadPerSec < 120.0, "integer division bug copied on purpose (finding U-3 in sim-plant.md)");
// Motor12VoltQuadEncoder.hpp kDeadbandRadPerSec: a command smaller than this writes 0 % duty.
constexpr double kWriteDeadbandRadPerSec = 0.05;  // [rad/s]
// Motor12VoltQuadEncoder.hpp kStictionDutyPercent: added to every non-zero duty.
constexpr double kWriteStictionDutyPercent = 2.0;  // [% of full duty]
// QuadEncoders.py: velocity = (position - position at the previous call) / 0.01, whatever the real
// time between calls was (:71), then a low-pass filter with alpha = 0.05 (:39).
constexpr double kEncoderAssumedPeriodS = 0.01;  // [s]
constexpr double kEncoderFilterAlpha = 0.05;     // [-]

// ---- Fixed in this code (spec §3.5), not parameters. --------------------------------------------
constexpr double kSubStepS = 0.001;   // [s] one RK4 step (why 1 ms: see step()); the IMU delay's
                                      //   resolution too
constexpr int kMaxSubStepsPerRead = 50;  // a read() never advances physics by more than 50 ms
constexpr int kMaxImuDelayMs = 1000;  // [ms] largest imu_delay_max_ms invalid_reason() accepts
// The mass matrix must have a determinant at least this fraction of a11 * a22 (a dimensionless
// margin, far below any sensible robot, above the rounding noise of a near-singular one).
constexpr double kMinMassMatrixDetFraction = 1e-9;  // [-]
// Width of the tanh that smooths the Coulomb friction sign (only used if coulomb_friction_Nm > 0).
constexpr double kCoulombSmoothingRadPerSec = 0.1;  // [rad/s]

// Every number the model uses. The PLUGIN does not use the defaults below: SimPlant.cpp requires
// every one of these as a <param> in description/ros2_control/rocko.ros2_control.xacro, whose
// values come from the args in description/urdf/rocko.urdf.xacro. The defaults here are what the unit test
// (tools/sim_plant_model_test.cpp) runs with. To change the SIMULATED ROBOT, edit rocko.urdf.xacro;
// verify_urdf.sh fails if a default there differs from the one here (sim/check_sim_config.py).
// Adding a parameter means touching four places: this struct, the args in rocko.urdf.xacro, the
// <param> list in rocko.ros2_control.xacro and the tables in SimPlant.cpp's read_parameters().
struct SimPlantParams
{
  // ===================== ESTIMATE, NOT MEASURED (spec §4.1) =====================================
  // Nothing in this block was weighed, measured or read off a part on ROCKO. Each line says where
  // the guess came from in the spec. Replace a value here AND in rocko.urdf.xacro once measured.
  // (The next line, motor_no_load_speed_rad_s, is the motor's real 1150 RPM in floating point; the
  // write-path constant kWriteMaxRadPerSec above is the code's integer-division copy of it.)
  double wheel_radius_m = 0.06;            // [m] r. Stack's value; issue #16 is still open (§4.2)
  double belt_ratio = 1.355;               // [-] N_b, gearbox output turns per wheel turn
  double wheel_mass_kg = 0.3;              // [kg] EACH wheel. URDF says "arbitrary"
  double body_mass_kg = 2.0;               // [kg] m_b, everything but the wheels. Invented
  double com_height_m = 0.20;              // [m] l, axle to body centre of mass. Invented
  double body_height_m = 0.40;             // [m] used only for the body inertia. Invented
  double body_depth_m = 0.10;              // [m] used only for the body inertia. Invented
  double gravity_m_s2 = 9.81;              // [m/s^2] standard gravity (not a guess)
  double motor_stall_torque_Nm = 0.774725;  // [N m] each motor, gearbox output, 12 V (7.9 kgf cm)
  double motor_no_load_speed_rad_s = 1150.0 / 60.0 * 2.0 * kPi;  // [rad/s] 1150 RPM = 120.428
  double motor_stall_current_A = 9.2;      // [A] only used to estimate friction
  double motor_no_load_current_A = 0.25;   // [A] only used to estimate friction (0 = no friction)
  double coulomb_friction_Nm = 0.0;        // [N m] off: unknown
  double battery_voltage_V = 12.0;         // [V] constant: no sag
  double motor_rated_voltage_V = 12.0;     // [V] the voltage the two motor figures are quoted at
  int imu_delay_ms = 20;                   // [ms] BNO055 fusion lag + service call (finding H-6)
  double imu_pitch_bias_deg = 0.0;         // [deg] error in "upright = 0"; 0 assumes 7.6 deg is
                                           //   exact
  double fallen_pitch_deg = 60.0;          // [deg] past this the body is "on the floor"; freeze lean
  // ================================================================================================

  // ---- Conventions and switches: assumptions, not physics (spec §2.4, §3, §4.1) ----
  double pitch_sign = -1.0;    // exported pitch = pitch_sign * lean toward sim-forward. -1: a
                               //   positive wheel command makes the exported pitch go UP
  double encoder_sign = +1.0;  // exported position/velocity = encoder_sign * motion toward sim-forward
  bool model_write_quirks = true;  // true: copy Motor12VoltQuadEncoder::write() exactly (§3.3)
  bool model_encoder_node = true;  // true: copy QuadEncoders.py, including finding U-2 (§3.7)
  int imu_delay_max_ms = 100;      // [ms] size of the delay buffer, allocated once

  // ---- Scenario and fault injection (spec §5). All off by default. Times count from release. ----
  double initial_pitch_deg = 0.0;  // [deg] starting lean, in the EXPORTED pitch convention
  bool disable_torque = false;     // true: motors produce no torque at all (no drive, no back-EMF)
  double imu_fault_start_s = 0.0;  // [s] start of an IMU outage
  double imu_fault_duration_s = 0.0;  // [s] its length; 0 = no outage
  double impulse_time_s = -1.0;    // [s] when a push happens; negative = never
  double impulse_angular_velocity_deg_per_s = 0.0;  // [deg/s] added once to the lean rate (exported
                                                    //   convention)
};

// Numbers worked out from the parameters (spec §3.2, §3.3). All in SI units.
struct DerivedParams
{
  double a11;  // [kg m^2] (M_w + m_b) r^2 + I_w : wheel-pair "inertia" including the body's mass
  double a12;  // [kg m^2] m_b r l               : coupling between wheel and body
  double a22;  // [kg m^2] m_b l^2 + I_b          : body inertia about the axle
  double mgl;  // [N m]    m_b g l                : gravity torque per unit sin(lean)
  double b;    // [N m s/rad] viscous friction, both wheels, on the wheel-to-body speed
};

inline DerivedParams derive(const SimPlantParams & p)
{
  const double r = p.wheel_radius_m;
  constexpr double wheel_count = 2.0;  // [-] left and right: two wheels, two motors
  const double wheels_mass = wheel_count * p.wheel_mass_kg;                 // [kg] all wheels
  const double wheels_inertia =                                             // [kg m^2] solid discs
    wheel_count * 0.5 * p.wheel_mass_kg * r * r;
  const double body_inertia =                                       // [kg m^2] uniform box
    p.body_mass_kg * (p.body_height_m * p.body_height_m + p.body_depth_m * p.body_depth_m) / 12.0;
  // Friction per motor from its no-load current: at no-load speed the motor shaft loses a torque
  // k_t * I_nl, so as a viscous friction on the motor shaft b_motor = k_t * I_nl / w_nl. Seen from
  // the wheel, the belt multiplies torque by N_b AND the motor shaft turns N_b times faster than
  // the wheel, so the friction torque on the wheel is N_b * b_motor * (N_b * w_wheel): N_b squared.
  const double k_t = p.motor_stall_torque_Nm / p.motor_stall_current_A;  // [N m/A]
  const double b_per_motor =
    p.belt_ratio * p.belt_ratio * k_t * p.motor_no_load_current_A / p.motor_no_load_speed_rad_s;

  DerivedParams d;
  d.a11 = (wheels_mass + p.body_mass_kg) * r * r + wheels_inertia;
  d.a12 = p.body_mass_kg * r * p.com_height_m;
  d.a22 = p.body_mass_kg * p.com_height_m * p.com_height_m + body_inertia;
  d.mgl = p.body_mass_kg * p.gravity_m_s2 * p.com_height_m;
  d.b = wheel_count * b_per_motor;
  return d;
}

// Checks what the plugin cannot run with. Returns nullptr if fine, else which parameter is wrong
// (`!(x > 0)` rather than `x <= 0`, so a NaN is refused too).
inline const char * invalid_reason(const SimPlantParams & p)
{
  if (!(p.wheel_radius_m > 0)) {return "wheel_radius_m must be > 0";}
  if (!(p.belt_ratio > 0)) {return "belt_ratio must be > 0";}
  if (!(p.wheel_mass_kg >= 0)) {return "wheel_mass_kg must be >= 0";}
  if (!(p.body_mass_kg > 0)) {return "body_mass_kg must be > 0";}
  if (!(p.com_height_m > 0)) {return "com_height_m must be > 0";}
  if (!(p.gravity_m_s2 > 0)) {return "gravity_m_s2 must be > 0";}
  if (!(p.motor_stall_torque_Nm >= 0)) {return "motor_stall_torque_Nm must be >= 0";}
  if (!(p.motor_no_load_speed_rad_s > 0)) {return "motor_no_load_speed_rad_s must be > 0";}
  if (!(p.motor_stall_current_A > 0)) {return "motor_stall_current_A must be > 0";}
  if (!(p.motor_no_load_current_A >= 0)) {return "motor_no_load_current_A must be >= 0";}
  if (!(p.coulomb_friction_Nm >= 0)) {return "coulomb_friction_Nm must be >= 0";}
  if (!(p.motor_rated_voltage_V > 0)) {return "motor_rated_voltage_V must be > 0";}
  if (p.pitch_sign != 1.0 && p.pitch_sign != -1.0) {return "pitch_sign must be 1 or -1";}
  if (p.encoder_sign != 1.0 && p.encoder_sign != -1.0) {return "encoder_sign must be 1 or -1";}
  if (p.imu_delay_ms < 0 || p.imu_delay_max_ms < 0 || p.imu_delay_ms > p.imu_delay_max_ms) {
    return "imu_delay_ms must be in [0, imu_delay_max_ms]";
  }
  if (!(p.fallen_pitch_deg > 0 && p.fallen_pitch_deg < 90)) {
    return "fallen_pitch_deg must be in (0, 90)";
  }
  if (!(p.body_height_m > 0)) {return "body_height_m must be > 0";}
  if (!(p.body_depth_m >= 0)) {return "body_depth_m must be >= 0";}
  if (!(p.battery_voltage_V > 0)) {return "battery_voltage_V must be > 0";}
  // The delay buffer holds one lean per millisecond; 1 s is far past any real IMU lag and stops a
  // typo like 100000 allocating a huge buffer.
  if (p.imu_delay_max_ms > kMaxImuDelayMs) {return "imu_delay_max_ms must be <= 1000";}
  // The equations of motion divide by det = a11 a22 - a12^2 cos^2(lean), which is smallest at lean
  // 0. If it is (nearly) zero the 2x2 mass matrix cannot be inverted and the state would blow up.
  const DerivedParams d = derive(p);
  if (!(d.a11 * d.a22 - d.a12 * d.a12 > kMinMassMatrixDetFraction * d.a11 * d.a22)) {
    return "the mass matrix is not invertible (a11*a22 - a12^2 must be > 0): check body_height_m, "
           "body_depth_m, wheel_mass_kg and com_height_m";
  }
  return nullptr;
}

// ---- The write path: a copy of Motor12VoltQuadEncoder::write() (spec §3.3) ----------------------
// Whole percent the real write() puts on the speed pin for a command [rad/s]: 0 for a non-finite
// command or one inside the deadband, else |cmd| / 119.381 * 100 + 2, clamped to [0, 100] and
// truncated toward zero (softPwmWrite takes an int).
inline int write_path_percent(double cmd_rad_s)
{
  if (!std::isfinite(cmd_rad_s) || std::abs(cmd_rad_s) < kWriteDeadbandRadPerSec) {
    return 0;
  }
  const double percent = std::clamp(
    std::abs(cmd_rad_s) / kWriteMaxRadPerSec * 100.0 + kWriteStictionDutyPercent, 0.0, 100.0);
  return static_cast<int>(percent);
}

// Signed duty in [-1, 1] the motor driver applies, + = toward sim-forward. With quirks off, an
// idealised write path: continuous, no deadband, no stiction offset.
inline double write_path_duty(double cmd_rad_s, bool model_write_quirks)
{
  if (!std::isfinite(cmd_rad_s)) {
    return 0.0;  // the real write() stops the motor on NaN/inf
  }
  if (!model_write_quirks) {
    return std::clamp(cmd_rad_s / kWriteMaxRadPerSec, -1.0, 1.0);
  }
  const double direction = cmd_rad_s >= 0 ? 1.0 : -1.0;  // the direction pin
  return direction * write_path_percent(cmd_rad_s) / 100.0;
}

// ---- The motor (spec §3.3) -----------------------------------------------------------------------
// Torque of ONE motor at its gearbox output [N m], + = toward sim-forward, for a signed duty and
// the gearbox output speed relative to the body omega_out [rad/s]. The usual DC-motor line
// tau = (k_t/R)(V - k_e w), written with the two datasheet numbers. duty = 0 still leaves the
// back-EMF term: the model assumes the driver brakes (shorts the motor) during PWM off-time.
inline double motor_output_torque(const SimPlantParams & p, double duty, double omega_out_rad_s)
{
  return p.motor_stall_torque_Nm *
         (duty * p.battery_voltage_V / p.motor_rated_voltage_V -
          omega_out_rad_s / p.motor_no_load_speed_rad_s);
}

// One wheel-pair-and-body state (spec §3.1).
struct PlantState
{
  double phi = 0.0;      // [rad] wheel angle vs the floor, + = rolling toward sim-forward
  double psi = 0.0;      // [rad] body lean from vertical, + = toward sim-forward
  double phi_dot = 0.0;  // [rad/s]
  double psi_dot = 0.0;  // [rad/s]
};

class SimPlantModel
{
public:
  // The only allocation: the lean history used for the IMU delay, sized imu_delay_max_ms + 1.
  // Call invalid_reason() first; this constructor trusts the parameters.
  explicit SimPlantModel(const SimPlantParams & params)
  : p_(params), d_(derive(params)),
    lean_history_(static_cast<std::size_t>(params.imu_delay_max_ms) + 1, 0.0)
  {
    reset();
  }

  // Back to the start of a scenario (spec §3.6 on_activate). Allocates nothing.
  void reset()
  {
    // reset() is the one place these members get their values: the constructor calls it, and so
    // does SimPlant::on_activate, so they carry no initialisers of their own.
    state_ = PlantState{};
    // initial_pitch_deg is in the exported convention; the physics lean has pitch_sign applied.
    state_.psi = deg_to_rad(p_.initial_pitch_deg) * p_.pitch_sign;
    std::fill(lean_history_.begin(), lean_history_.end(), state_.psi);
    history_head_ = 0;
    time_s_ = 0.0;
    duty_left_ = 0.0;
    duty_right_ = 0.0;
    impulse_applied_ = false;
    fallen_ = false;
    encoder_filter_ = 0.0;
    encoder_last_position_m_ = position_m();
    left_velocity_m_s_ = 0.0;
    right_velocity_m_s_ = 0.0;
  }

  // What write() does: run each wheel's command [rad/s] through the write-path copy and hold the
  // duty until the next call (a zero-order hold, as softPwm does on the robot).
  void set_wheel_commands(double left_rad_s, double right_rad_s)
  {
    duty_left_ = write_path_duty(left_rad_s, p_.model_write_quirks);
    duty_right_ = write_path_duty(right_rad_s, p_.model_write_quirks);
  }

  // Advance the physics by one sub-step of kSubStepS (spec §3.5, §3.6 read() step 1).
  void step()
  {
    if (!impulse_applied_ && p_.impulse_time_s >= 0.0 && time_s_ >= p_.impulse_time_s) {
      state_.psi_dot += deg_to_rad(p_.impulse_angular_velocity_deg_per_s) * p_.pitch_sign;
      impulse_applied_ = true;
    }

    // Classical 4th-order Runge-Kutta. Why: at 1 ms it is accurate enough that a disagreement
    // between this model and a hand calculation is never the integrator's fault (spec §3.5: the
    // fall time changes by less than 0.01 ms against a 100x smaller step). The duties are held
    // constant over the step because write() runs once per 10 ms, so they change only between
    // read()s.
    const double h = kSubStepS;
    const PlantState k1 = derivative(state_);
    const PlantState k2 = derivative(add_scaled(state_, k1, h / 2));
    const PlantState k3 = derivative(add_scaled(state_, k2, h / 2));
    const PlantState k4 = derivative(add_scaled(state_, k3, h));
    state_.phi += h / 6 * (k1.phi + 2 * k2.phi + 2 * k3.phi + k4.phi);
    state_.psi += h / 6 * (k1.psi + 2 * k2.psi + 2 * k3.psi + k4.psi);
    state_.phi_dot += h / 6 * (k1.phi_dot + 2 * k2.phi_dot + 2 * k3.phi_dot + k4.phi_dot);
    state_.psi_dot += h / 6 * (k1.psi_dot + 2 * k2.psi_dot + 2 * k3.psi_dot + k4.psi_dot);
    time_s_ += h;

    // Past fallen_pitch_deg the pendulum equations mean nothing (the body would be on the floor).
    // Freeze the lean; the wheels keep integrating.
    if (!fallen_ && std::abs(state_.psi) >= deg_to_rad(p_.fallen_pitch_deg)) {
      fallen_ = true;
    }
    if (fallen_) {
      state_.psi_dot = 0.0;
    }

    history_head_ = (history_head_ + 1) % lean_history_.size();
    lean_history_[history_head_] = state_.psi;
  }

  // Call once per read(), after the sub-steps: the encoder node copy (spec §3.7).
  void update_encoder_outputs()
  {
    if (!p_.model_encoder_node) {
      left_velocity_m_s_ = p_.encoder_sign * p_.wheel_radius_m * (state_.phi_dot - state_.psi_dot);
      right_velocity_m_s_ = left_velocity_m_s_;
      return;
    }
    // Finding U-2: both motor plugins call ONE service, one after the other, in the same cycle.
    // The first call sees ~10 ms of motion; the second sees none. Both divide by 0.01 s, and both
    // feed the SAME low-pass filter. So the two joints read about 0.51 and 0.49 of the true speed.
    const double position_now_m = position_m();
    const double first_call = (position_now_m - encoder_last_position_m_) / kEncoderAssumedPeriodS;
    encoder_filter_ = (1 - kEncoderFilterAlpha) * encoder_filter_ + kEncoderFilterAlpha * first_call;
    left_velocity_m_s_ = encoder_filter_;
    const double second_call = 0.0;  // no motion since the first call
    encoder_filter_ = (1 - kEncoderFilterAlpha) * encoder_filter_ + kEncoderFilterAlpha * second_call;
    right_velocity_m_s_ = encoder_filter_;
    encoder_last_position_m_ = position_now_m;
  }

  // ---- Outputs, in the units the real hardware exports (spec §3.7) ----
  // Pitch as the IMU would report it [deg]: the lean imu_delay_ms ago, sign and bias applied.
  double imu_pitch_deg() const
  {
    const std::size_t n = lean_history_.size();
    const std::size_t delay = static_cast<std::size_t>(p_.imu_delay_ms);
    const double delayed_lean = lean_history_[(history_head_ + n - delay) % n];
    return p_.pitch_sign * rad_to_deg(delayed_lean) + p_.imu_pitch_bias_deg;
  }
  // Pitch rate as the gyro would report it [deg/s], same sign as imu_pitch_deg() (finding H-6).
  // The CURRENT lean rate, NOT delayed by imu_delay_ms: the gyro's rate skips the fusion filter that
  // the delay stands in for (spec §3.7). No bias term: a constant angle error has no rate.
  double imu_pitch_rate_deg_s() const
  {
    return p_.pitch_sign * rad_to_deg(state_.psi_dot);
  }
  // Chassis-relative wheel travel [m] (what an encoder on the motor measures).
  double position_m() const
  {
    return p_.encoder_sign * p_.wheel_radius_m * (state_.phi - state_.psi);
  }
  double left_velocity_m_s() const {return left_velocity_m_s_;}
  double right_velocity_m_s() const {return right_velocity_m_s_;}

  // ---- For the plugin's checks and the unit test ----
  double time_s() const {return time_s_;}
  bool fallen() const {return fallen_;}
  const PlantState & state() const {return state_;}
  const DerivedParams & derived() const {return d_;}
  const SimPlantParams & params() const {return p_;}
  bool state_is_finite() const
  {
    return std::isfinite(state_.phi) && std::isfinite(state_.psi) &&
           std::isfinite(state_.phi_dot) && std::isfinite(state_.psi_dot);
  }
  // Kinetic + potential energy [J] (spec §3.2). Constant when nothing does work (unit test #9).
  double energy_J() const
  {
    const PlantState & s = state_;
    const double kinetic = 0.5 * d_.a11 * s.phi_dot * s.phi_dot +
      d_.a12 * std::cos(s.psi) * s.phi_dot * s.psi_dot + 0.5 * d_.a22 * s.psi_dot * s.psi_dot;
    return kinetic + d_.mgl * std::cos(s.psi);
  }
  // Total torque between body and wheels [N m], + = on the wheels toward sim-forward (spec §3.3).
  double net_torque_Nm(const PlantState & s) const
  {
    const double rel = s.phi_dot - s.psi_dot;  // [rad/s] wheel speed relative to the body
    double motors = 0.0;
    if (!p_.disable_torque) {
      const double omega_out = p_.belt_ratio * rel;  // gearbox output turns faster than the wheel
      motors = p_.belt_ratio * (motor_output_torque(p_, duty_left_, omega_out) +
                                motor_output_torque(p_, duty_right_, omega_out));
    }
    const double friction =
      d_.b * rel + p_.coulomb_friction_Nm * std::tanh(rel / kCoulombSmoothingRadPerSec);
    return motors - friction;
  }

private:
  static PlantState add_scaled(const PlantState & s, const PlantState & ds, double h)
  {
    return PlantState{s.phi + h * ds.phi, s.psi + h * ds.psi,
      s.phi_dot + h * ds.phi_dot, s.psi_dot + h * ds.psi_dot};
  }

  // The equations of motion (spec §3.2), solved for the two accelerations. The returned struct
  // holds d/dt of each state: (phi_dot, psi_dot, phi_ddot, psi_ddot).
  //   a11 phi''          + a12 cos(psi) psi'' =  tau + a12 sin(psi) psi'^2   (wheel)
  //   a12 cos(psi) phi'' + a22 psi''          = -tau + m_b g l sin(psi)      (body)
  // The motor torque acts BETWEEN body and wheel: +tau on the wheels, -tau on the body.
  PlantState derivative(const PlantState & s) const
  {
    const double tau = net_torque_Nm(s);
    if (fallen_) {
      // Body lying still: only the wheels move.
      return PlantState{s.phi_dot, 0.0, tau / d_.a11, 0.0};
    }
    const double c = std::cos(s.psi);
    const double rhs_wheel = tau + d_.a12 * std::sin(s.psi) * s.psi_dot * s.psi_dot;
    const double rhs_body = -tau + d_.mgl * std::sin(s.psi);
    // > 0 because |cos| <= 1 and invalid_reason() refuses parameters where a11 a22 - a12^2 is not
    // safely positive.
    const double det = d_.a11 * d_.a22 - d_.a12 * d_.a12 * c * c;
    const double phi_ddot = (d_.a22 * rhs_wheel - d_.a12 * c * rhs_body) / det;
    const double psi_ddot = (d_.a11 * rhs_body - d_.a12 * c * rhs_wheel) / det;
    return PlantState{s.phi_dot, s.psi_dot, phi_ddot, psi_ddot};
  }

  SimPlantParams p_;
  DerivedParams d_;
  // Everything below is set by reset(), which the constructor calls.
  PlantState state_;
  double time_s_;     // [s] sim time since reset: the sum of sub-steps, not wall-clock time
  double duty_left_;  // [-1, 1] held from the last set_wheel_commands()
  double duty_right_;
  bool impulse_applied_;
  bool fallen_;
  // Lean [rad] at each of the last imu_delay_max_ms + 1 sub-steps, a ring; history_head_ = newest.
  std::vector<double> lean_history_;
  std::size_t history_head_;
  double encoder_filter_;           // [m/s] the one filter both encoder calls share (U-2)
  double encoder_last_position_m_;  // [m] position at the previous service call
  double left_velocity_m_s_;        // [m/s] exported velocity states
  double right_velocity_m_s_;
};

// ---- The simulated IMU read, with the finding C-2 policy of ICM20948::read() (spec §5) -----------
// A failed read keeps the last good pitch and pitch rate (never 0.0; NaN only if there was never a
// good read) and is counted; once ImuFaultTracker trips, read() must return ERROR. A good read always
// returns OK, as in ICM20948::read(), where one read gives both values.
class SimImuOutput
{
public:
  // Same value as kMaxConsecutiveImuFailures in ICM20948.hpp. That constant is private to a class
  // that needs ROS headers, so it cannot be used here; sim/check_sim_config.py (run by
  // verify_urdf.sh) fails if the two numbers differ.
  static constexpr int kMaxConsecutiveFailures = 5;  // [reads]

  void reset()
  {
    pitch_deg_ = std::numeric_limits<double>::quiet_NaN();
    pitch_rate_deg_s_ = std::numeric_limits<double>::quiet_NaN();
    tracker_ = ImuFaultTracker(kMaxConsecutiveFailures);
  }

  // Returns false when read() must return ERROR.
  bool update(bool read_ok, double measured_pitch_deg, double measured_pitch_rate_deg_s)
  {
    if (read_ok) {
      pitch_deg_ = measured_pitch_deg;
      pitch_rate_deg_s_ = measured_pitch_rate_deg_s;
    }
    tracker_.record(read_ok);
    return read_ok || !tracker_.tripped();
  }

  double pitch_deg() const {return pitch_deg_;}
  double pitch_rate_deg_s() const {return pitch_rate_deg_s_;}
  const ImuFaultTracker & tracker() const {return tracker_;}

private:
  double pitch_deg_ = std::numeric_limits<double>::quiet_NaN();         // [deg] exported pitch
  double pitch_rate_deg_s_ = std::numeric_limits<double>::quiet_NaN();  // [deg/s] exported rate
  ImuFaultTracker tracker_{kMaxConsecutiveFailures};
};

}  // namespace sim
}  // namespace rocko_env

#endif  // ROCKO_ENV_SIM_PLANT_MODEL
