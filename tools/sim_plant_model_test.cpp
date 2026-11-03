// Host-side test of rocko_env::sim::SimPlantModel, the physics inside the SimPlant plugin.
// No ROS needed; it runs on a laptop or in the container.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include -I rocko_env/rocko_env/hardware/sensors/icm20948/include tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test
// (One line: a backslash at the end of a // comment line makes the compiler warn.)
// Exit status 0 and the line "all SimPlantModel tests passed" mean success; every failed check is
// printed with what was expected, and the exit status is 1.
//
// What it checks: that the model computes what docs/architecture/sim-plant.md §7.1 says it should
// (the numbers in brackets, #1 ... #17, are that table's rows). It tests the MODEL AS SPECIFIED,
// with its ESTIMATED parameters. It says nothing about the real robot.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "rocko_env/SimPlantModel.hpp"

using rocko_env::sim::SimImuOutput;
using rocko_env::sim::SimPlantModel;
using rocko_env::sim::SimPlantParams;
using rocko_env::sim::deg_to_rad;
using rocko_env::sim::rad_to_deg;
using rocko_env::sim::invalid_reason;
using rocko_env::sim::kSubStepS;

namespace
{
int failures = 0;

// near(): |got - expected| <= tolerance. Prints every check, so a red run shows which one and why.
void near(const char * what, double got, double expected, double tolerance)
{
  const bool ok = std::abs(got - expected) <= tolerance;
  std::printf("%s  %-62s got %.6g, expected %.6g +/- %.3g\n", ok ? "ok  " : "FAIL", what, got,
    expected, tolerance);
  if (!ok) {++failures;}
}

void check(const char * what, bool ok)
{
  std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) {++failures;}
}

// Parameters with friction removed (the no-load current is what sets the friction b).
SimPlantParams no_friction(SimPlantParams p)
{
  p.motor_no_load_current_A = 0.0;
  return p;
}

SimPlantParams torque_off(SimPlantParams p)
{
  p.disable_torque = true;
  return p;
}

// Growth rate [1/s] of the lean from a tiny start, measured between t1 and t2 [s] of simulated
// time, once the growing mode dominates. Linear regime: the lean stays far below 1 degree.
double growth_rate(const SimPlantParams & p, double t1, double t2)
{
  SimPlantModel m(p);
  m.set_wheel_commands(0.0, 0.0);
  double psi_t1 = 0.0;
  const int n1 = static_cast<int>(std::lround(t1 / kSubStepS));
  const int n2 = static_cast<int>(std::lround(t2 / kSubStepS));
  for (int i = 1; i <= n2; ++i) {
    m.step();
    if (i == n1) {psi_t1 = m.state().psi;}
  }
  return std::log(m.state().psi / psi_t1) / (t2 - t1);
}

struct Crossing
{
  double time_s = -1.0;        // [s] when |true lean| first reached the angle (interpolated)
  double lean_rate_deg_s = 0;  // [deg/s] at that moment
  double axle_m = 0;           // [m] axle position r * phi at that moment
};

// Model only, no delay, no ROS (#6, #8): from rest at initial_pitch_deg until |lean| >= angle_deg.
Crossing fall_time(const SimPlantParams & p, double angle_deg)
{
  SimPlantModel m(p);
  const double target = deg_to_rad(angle_deg);
  for (int i = 0; i < 5000; ++i) {  // 5 s at most
    const auto before = m.state();
    const double t_before = m.time_s();
    m.step();
    const auto after = m.state();
    if (std::abs(after.psi) >= target) {
      // Linear interpolation inside the 1 ms step.
      const double f = (target - std::abs(before.psi)) / (std::abs(after.psi) - std::abs(before.psi));
      Crossing c;
      c.time_s = t_before + f * kSubStepS;
      c.lean_rate_deg_s = rad_to_deg(before.psi_dot + f * (after.psi_dot - before.psi_dot));
      c.axle_m = p.wheel_radius_m * (before.phi + f * (after.phi - before.phi));
      return c;
    }
  }
  return Crossing{};
}

SimPlantParams start_at(SimPlantParams p, double pitch_deg)
{
  p.initial_pitch_deg = pitch_deg;
  return p;
}

}  // namespace

int main()
{
  const SimPlantParams nominal;  // the spec's estimates

  // #1 Inertia terms.
  {
    const auto d = rocko_env::sim::derive(nominal);
    near("#1 a11 [kg m^2]", d.a11, 0.01044, 0.01044e-6);
    near("#1 a12 [kg m^2]", d.a12, 0.024, 0.024e-6);
    near("#1 a22 [kg m^2]", d.a22, 0.1083333333, 0.1083333333e-6);
    near("#1 D = a11 a22 - a12^2 [kg^2 m^4]", d.a11 * d.a22 - d.a12 * d.a12, 5.55e-4, 5.55e-4 * 1e-6);
    near("    friction b, both wheels [N m s/rad]", d.b, 6.42e-4, 0.01e-4);
  }

  // #2 Open-loop pole, torque off, no friction: closed form, and measured from the simulation
  // (from rest the linear lean is psi0 cosh(lambda t)).
  {
    const auto d = rocko_env::sim::derive(nominal);
    const double D = d.a11 * d.a22 - d.a12 * d.a12;
    near("#2 closed form sqrt(a11 m g l / D) [1/s]", std::sqrt(d.a11 * d.mgl / D), 8.5915, 1e-3);
    SimPlantModel m(start_at(torque_off(no_friction(nominal)), 1e-5));
    const double psi0 = m.state().psi;
    for (int i = 0; i < 1000; ++i) {m.step();}
    near("#2 pole measured from 1 s of simulation [1/s]", std::acosh(m.state().psi / psi0) / 1.0,
      8.5915, 1e-3);
  }

  // #3 Torque off, with friction b: the growing pole.
  near("#3 torque-off pole with friction [1/s]",
    growth_rate(start_at(torque_off(nominal), 1e-8), 1.5, 2.0), 8.5265, 1e-3);

  // #4 Motor braked (duty 0, back-EMF present): the growing pole.
  near("#4 braked pole, duty 0 [1/s]", growth_rate(start_at(nominal, 1e-8), 2.0, 2.5), 6.9306, 1e-3);

  // #5 Free wheels let the body fall faster than a fixed pivot would.
  {
    const auto d = rocko_env::sim::derive(nominal);
    const double D = d.a11 * d.a22 - d.a12 * d.a12;
    const double free_wheels = std::sqrt(d.a11 * d.mgl / D);
    const double point_mass = std::sqrt(nominal.gravity_m_s2 / nominal.com_height_m);
    const double body_on_pivot = std::sqrt(d.mgl / d.a22);
    near("#5 point mass on a pivot sqrt(g/l) [1/s]", point_mass, 7.0036, 1e-3);
    near("#5 body on a pivot [1/s]", body_on_pivot, 6.0184, 1e-3);
    check("#5 ordering free wheels > point mass > body on pivot",
      free_wheels > point_mass && point_mass > body_on_pivot);
  }

  // #6 and #8 Fall time 2 -> 35 deg, torque off, model only.
  {
    const Crossing with_b = fall_time(start_at(torque_off(nominal), 2.0), 35.0);
    const Crossing without_b = fall_time(start_at(torque_off(no_friction(nominal)), 2.0), 35.0);
    near("#6 fall time 2->35 deg, torque off, with friction [s]", with_b.time_s, 0.4273, 0.001);
    near("#6 fall time 2->35 deg, torque off, no friction [s]", without_b.time_s, 0.4250, 0.001);
    // The spec states #8 for a lean toward sim-forward. With pitch_sign = -1, initial_pitch_deg = 2
    // is a TRUE lean of -2 deg (away from sim-forward), so the motion is the mirror image. Measure
    // both quantities along the lean: lean_dir = -1 here.
    const double lean_dir = nominal.pitch_sign;  // sign of the true lean for a +2 deg start
    near("#8 lean rate at 35 deg, along the lean [deg/s]", with_b.lean_rate_deg_s * lean_dir, 254.6,
      2.546);
    near("#8 axle position at 35 deg, along the lean [m] (wheels roll the OTHER way)",
      with_b.axle_m * lean_dir, -0.074, 0.00074);
    // Not in the spec's table: the same fall with the motor braked (duty 0), for verify_sim.sh's
    // choice of time limit. Printed, not asserted.
    const Crossing braked = fall_time(start_at(nominal, 2.0), 35.0);
    std::printf("info  fall time 2->35 deg with the motor braked at duty 0: %.4f s\n", braked.time_s);
  }

  // Sweep corners (spec §7.1, second table): fall time with friction, torque off.
  {
    SimPlantParams p = start_at(torque_off(nominal), 2.0);
    p.body_mass_kg = 2.0 * 0.7;
    near("    sweep: body mass x0.7, fall time [s]", fall_time(p, 35).time_s, 0.4506, 0.001);
    p.body_mass_kg = 2.0 * 1.3;
    near("    sweep: body mass x1.3, fall time [s]", fall_time(p, 35).time_s, 0.4112, 0.001);
    p = start_at(torque_off(nominal), 2.0);
    p.com_height_m = 0.20 * 0.7;
    // The spec holds the body inertia I_b fixed while l moves. derive() computes I_b from the box
    // size, which does not depend on l, so this matches.
    near("    sweep: CoM height x0.7, fall time [s]", fall_time(p, 35).time_s, 0.4417, 0.001);
    p.com_height_m = 0.20 * 1.3;
    near("    sweep: CoM height x1.3, fall time [s]", fall_time(p, 35).time_s, 0.4340, 0.001);
  }

  // #7 The same fall seen through the plugin's 100 Hz cycle with the 20 ms IMU delay: the first
  // read() whose exported |pitch| >= 35 deg. Each read() advances 10 sub-steps.
  {
    SimPlantModel m(start_at(torque_off(nominal), 2.0));
    double first_read_s = -1.0;
    for (int cycle = 1; cycle <= 200 && first_read_s < 0; ++cycle) {
      for (int i = 0; i < 10; ++i) {m.step();}
      if (std::abs(m.imu_pitch_deg()) >= 35.0) {first_read_s = m.time_s();}
    }
    near("#7 first 100 Hz read with |pitch| >= 35 deg, 20 ms delay [s]", first_read_s, 0.45, 0.01);
  }

  // #9 Energy is conserved when nothing does work (no friction, torque off), 2 -> 35 deg.
  {
    SimPlantModel m(start_at(torque_off(no_friction(nominal)), 2.0));
    const double e0 = m.energy_J();
    double worst = 0.0;
    // Bounded (5 s), so a broken model that never falls fails the check instead of hanging here.
    for (int i = 0; i < 5000 && std::abs(m.state().psi) < deg_to_rad(35.0); ++i) {
      m.step();
      worst = std::max(worst, std::abs(m.energy_J() - e0) / std::abs(e0));
    }
    check("#9 setup: the body reached 35 deg within 5 s", std::abs(m.state().psi) >= deg_to_rad(35.0));
    near("#9 energy drift, relative", worst, 0.0, 1e-9);
  }

  // #10 Exactly upright, at rest, command 0: nothing moves, bitwise, for 10 s.
  {
    SimPlantModel m(nominal);
    m.set_wheel_commands(0.0, 0.0);
    bool all_zero = true;
    for (int i = 0; i < 10000; ++i) {
      m.step();
      const auto s = m.state();
      all_zero = all_zero && s.phi == 0.0 && s.psi == 0.0 && s.phi_dot == 0.0 && s.psi_dot == 0.0;
    }
    check("#10 upright, at rest, command 0: state exactly 0 for 10 s", all_zero);
  }

  // #11 Write path: command [rad/s] -> whole percent on the speed pin, as the NEW
  // Motor12VoltQuadEncoder::write() does it (deadband 0.05 rad/s, +2 % stiction offset, clamp).
  {
    using rocko_env::sim::write_path_percent;
    using rocko_env::sim::write_path_duty;
    near("#11 MAX_RAD_PER_SEC as the code computes it (19 * 2 pi)",
      rocko_env::sim::kWriteMaxRadPerSec, 119.381, 0.001);
    near("#11 cmd 0 -> 0 %", write_path_percent(0.0), 0, 0);
    near("#11 cmd 0.049 (inside the deadband) -> 0 %", write_path_percent(0.049), 0, 0);
    near("#11 cmd 0.05 -> 2 % (stiction offset alone)", write_path_percent(0.05), 2, 0);
    near("#11 cmd 1.0 -> 2 %", write_path_percent(1.0), 2, 0);
    near("#11 cmd 1.2 -> 3 %", write_path_percent(1.2), 3, 0);
    near("#11 cmd 5.0 -> 6 %", write_path_percent(5.0), 6, 0);
    near("#11 cmd 60 -> 52 %", write_path_percent(60.0), 52, 0);
    near("#11 cmd 119.381 -> 100 % (102 clamped)", write_path_percent(119.381), 100, 0);
    near("#11 cmd 1e9 -> 100 %", write_path_percent(1e9), 100, 0);
    near("#11 cmd -5.0 -> 6 %", write_path_percent(-5.0), 6, 0);
    near("#11 cmd -5.0 -> duty -0.06 (direction negative)", write_path_duty(-5.0, true), -0.06, 1e-12);
    near("#11 cmd NaN -> duty 0", write_path_duty(std::nan(""), true), 0.0, 0.0);
    near("#11 quirks off: cmd 59.69 -> duty 0.5", write_path_duty(59.6902604, false), 0.5, 1e-6);
  }

  // #12 The motor function alone.
  {
    using rocko_env::sim::motor_output_torque;
    near("#12 tau(duty 1, speed 0) = stall torque [N m]", motor_output_torque(nominal, 1.0, 0.0),
      nominal.motor_stall_torque_Nm, 1e-12);
    near("#12 tau(duty 1, no-load speed) = 0 [N m]",
      motor_output_torque(nominal, 1.0, nominal.motor_no_load_speed_rad_s), 0.0, 1e-12);
  }

  // #13 Encoder replica (finding U-2) at a constant chassis-relative speed. To get a constant
  // speed out of the model: torque off, no friction, let the body fall until it is "on the floor"
  // (lean frozen); after that nothing acts on the wheels and they coast at a constant rate.
  // Then compare the exported velocities with the true speed.
  {
    SimPlantParams q = torque_off(no_friction(nominal));
    q.initial_pitch_deg = 2.0;
    SimPlantModel m(q);
    // Fall to 60 deg (then the lean is frozen). Bounded at 5 s so a model that never falls fails.
    for (int i = 0; i < 5000 && !m.fallen(); ++i) {m.step();}
    check("#13 setup: the body fell within 5 s", m.fallen());
    m.update_encoder_outputs();      // first read after the fall: discard
    for (int cycle = 0; cycle < 300; ++cycle) {  // 3 s, about 30 time constants
      for (int i = 0; i < 10; ++i) {m.step();}
      m.update_encoder_outputs();
    }
    const double v = q.encoder_sign * q.wheel_radius_m * (m.state().phi_dot - m.state().psi_dot);
    check("#13 model: wheels coasting at a non-zero speed", std::abs(v) > 0.01);
    near("#13 model: left joint velocity / true speed", m.left_velocity_m_s() / v, 0.5128, 1e-3);
    near("#13 model: right joint velocity / true speed", m.right_velocity_m_s() / v, 0.4872, 1e-3);

    q.model_encoder_node = false;
    SimPlantModel ideal(q);
    for (int i = 0; i < 5000 && !ideal.fallen(); ++i) {ideal.step();}
    ideal.update_encoder_outputs();
    const double vi = q.wheel_radius_m * (ideal.state().phi_dot - ideal.state().psi_dot);
    near("#13 model_encoder_node=false: velocity = true speed [m/s]", ideal.left_velocity_m_s(), vi,
      1e-12);
  }

  // #14 IMU delay: a step in the true lean appears in the exported pitch exactly imu_delay_ms later.
  // The lean starts moving at the first sub-step (torque off, 2 deg); the exported pitch must stay
  // at exactly 2.0 for the first 20 sub-steps and change at the 21st.
  {
    SimPlantModel m(start_at(torque_off(nominal), 2.0));
    const double start = m.imu_pitch_deg();
    int first_change = -1;
    for (int i = 1; i <= 100 && first_change < 0; ++i) {
      m.step();
      if (m.imu_pitch_deg() != start) {first_change = i;}
    }
    near("#14 exported pitch at t = 0 equals initial_pitch_deg [deg]", start, 2.0, 1e-12);
    near("#14 first sub-step whose exported pitch moved (delay 20 ms)", first_change, 21, 0);
  }

  // #15 Signs, default pitch_sign = -1.
  {
    SimPlantModel m(nominal);
    m.set_wheel_commands(10.0, 10.0);  // positive command on both wheels, from upright
    for (int i = 0; i < 50; ++i) {m.step();}
    check("#15 positive command: wheels roll toward sim-forward (phi > 0)", m.state().phi > 0);
    check("#15 positive command: body tips AWAY from sim-forward (psi < 0)", m.state().psi < 0);
    check("#15 positive command: exported pitch RISES", m.imu_pitch_deg() > 0);

    // The size of that push, not only its sign: the input column B of spec §3.4 (wheel and lean
    // acceleration per rad/s of command, from upright and at rest, idealised write path). One 1 ms
    // step from rest gives rate ~ acceleration x 1 ms; back-EMF builds up within the step, hence 1 %.
    SimPlantParams ideal_write = nominal;
    ideal_write.model_write_quirks = false;
    SimPlantModel push(ideal_write);
    const double u = 10.0;  // [rad/s] command on both wheels
    push.set_wheel_commands(u, u);
    push.step();
    near("#15 B: wheel acceleration per rad/s of command [rad/s^2]",
      push.state().phi_dot / kSubStepS / u, 4.1933, 4.1933 * 0.01);
    near("#15 B: lean acceleration per rad/s of command [rad/s^2]",
      push.state().psi_dot / kSubStepS / u, -1.0913, 1.0913 * 0.01);

    SimPlantModel f(start_at(torque_off(nominal), 2.0));
    for (int i = 0; i < 600; ++i) {f.step();}
    check("#15 torque off from +2 deg: exported pitch grows past +35",
      f.imu_pitch_deg() >= 35.0);

    // disable_torque really removes the motor: the same command changes nothing.
    SimPlantModel a(start_at(torque_off(nominal), 2.0));
    SimPlantModel b(start_at(torque_off(nominal), 2.0));
    b.set_wheel_commands(50.0, 50.0);
    for (int i = 0; i < 300; ++i) {a.step(); b.step();}
    check("#15 disable_torque: a 50 rad/s command leaves the motion unchanged",
      a.state().psi == b.state().psi && a.state().phi == b.state().phi);
  }

  // #16 IMU fault policy: an outage covering 5 reads.
  {
    SimImuOutput imu;
    imu.reset();
    check("#16 pitch is NaN before any good read", std::isnan(imu.pitch_deg()));
    check("#16 pitch rate is NaN before any good read", std::isnan(imu.pitch_rate_deg_s()));
    check("#16 good read returns OK", imu.update(true, 3.5, -12.25));
    bool ok_1_to_4 = true, held = true;
    for (int i = 1; i <= 4; ++i) {
      ok_1_to_4 = ok_1_to_4 && imu.update(false, 0.0, 0.0);
      held = held && imu.pitch_deg() == 3.5 && imu.pitch_rate_deg_s() == -12.25;
    }
    check("#16 failed reads 1-4 return OK", ok_1_to_4);
    check("#16 failed reads hold the last good pitch and pitch rate (never 0.0)", held);
    check("#16 failed read 5 returns ERROR", !imu.update(false, 0.0, 0.0));
    check("#16 pitch still held after the trip", imu.pitch_deg() == 3.5);
    SimImuOutput blind;
    blind.reset();
    blind.update(false, 0.0, 0.0);
    check("#16 a failure before any good read leaves NaN, not 0.0", std::isnan(blind.pitch_deg()));
    check("#16 ... and leaves the pitch rate NaN, not 0.0", std::isnan(blind.pitch_rate_deg_s()));
  }

  // #18 Pitch rate (finding H-6, spec §3.7): imu_pitch_rate_deg_s() = pitch_sign * degrees(psi_dot),
  // the CURRENT rate, with NO imu_delay_ms. It must have the same sign as the exported pitch.
  {
    // No delay: with the default 20 ms IMU delay, torque off from +2 deg. After 10 ms the exported
    // pitch has not moved yet (it shows the lean of 20 ms ago), but the rate already shows the fall.
    SimPlantModel m(start_at(torque_off(nominal), 2.0));
    check("#18 setup: the default IMU delay is 20 ms", nominal.imu_delay_ms == 20);
    near("#18 pitch rate at rest is 0 [deg/s]", m.imu_pitch_rate_deg_s(), 0.0, 0.0);
    for (int i = 0; i < 10; ++i) {m.step();}
    near("#18 after 10 ms the exported (delayed) pitch has not moved [deg]", m.imu_pitch_deg(), 2.0,
      0.0);
    near("#18 after 10 ms the rate is the CURRENT one, pitch_sign * deg(psi_dot) [deg/s]",
      m.imu_pitch_rate_deg_s(), nominal.pitch_sign * rad_to_deg(m.state().psi_dot), 1e-12);
    check("#18 after 10 ms the rate is already non-zero (no delay)", m.imu_pitch_rate_deg_s() != 0.0);

    // Same sign as the exported pitch: falling from +2 deg, pitch grows, so the rate is positive.
    check("#18 falling from +2 deg (pitch_sign -1): rate > 0", m.imu_pitch_rate_deg_s() > 0.0);
    SimPlantParams plus = start_at(torque_off(nominal), 2.0);
    plus.pitch_sign = +1.0;
    SimPlantModel mp(plus);
    for (int i = 0; i < 10; ++i) {mp.step();}
    check("#18 falling from +2 deg (pitch_sign +1): rate > 0", mp.imu_pitch_rate_deg_s() > 0.0);

    // The rate is the derivative of the exported pitch: with the delay set to 0, the change of
    // pitch over one 1 ms sub-step equals the mean of the rates at its two ends (trapezoid rule).
    SimPlantParams no_delay = start_at(torque_off(nominal), 2.0);
    no_delay.imu_delay_ms = 0;
    SimPlantModel d(no_delay);
    for (int i = 0; i < 200; ++i) {d.step();}
    const double pitch_before = d.imu_pitch_deg();
    const double rate_before = d.imu_pitch_rate_deg_s();
    d.step();
    near("#18 with no delay, d(pitch)/dt = rate over one sub-step [deg/s]",
      (d.imu_pitch_deg() - pitch_before) / kSubStepS, (rate_before + d.imu_pitch_rate_deg_s()) / 2,
      1e-3 * std::abs(rate_before));
  }

  // #17 Parameter validation: invalid_reason() accepts the defaults and refuses what would make the
  // model divide by zero, allocate absurdly, or run with a nonsense supply.
  {
    const SimPlantParams good;
    check("#17 the default parameters are accepted", invalid_reason(good) == nullptr);

    SimPlantParams p = good;
    p.imu_delay_max_ms = 1000;
    check("#17 imu_delay_max_ms = 1000 is accepted (the largest allowed)", invalid_reason(p) == nullptr);
    p.imu_delay_max_ms = 1001;
    check("#17 imu_delay_max_ms = 1001 is refused", invalid_reason(p) != nullptr);

    p = good;
    p.body_height_m = 0.0;
    check("#17 body_height_m = 0 is refused", invalid_reason(p) != nullptr);
    p = good;
    p.body_depth_m = -0.1;
    check("#17 body_depth_m < 0 is refused", invalid_reason(p) != nullptr);
    p = good;
    p.body_depth_m = 0.0;
    check("#17 body_depth_m = 0 is accepted", invalid_reason(p) == nullptr);
    p = good;
    p.battery_voltage_V = 0.0;
    check("#17 battery_voltage_V = 0 is refused", invalid_reason(p) != nullptr);
    p.battery_voltage_V = std::nan("");
    check("#17 battery_voltage_V = NaN is refused", invalid_reason(p) != nullptr);

    // A body with no wheel mass and (almost) no inertia of its own makes a11 a22 - a12^2 vanish:
    // the mass matrix cannot be inverted.
    p = good;
    p.wheel_mass_kg = 0.0;
    p.body_height_m = 1e-9;
    p.body_depth_m = 0.0;
    const char * why = invalid_reason(p);
    check("#17 a singular mass matrix is refused, with that reason",
      why != nullptr && std::strstr(why, "mass matrix") != nullptr);
  }

  if (failures == 0) {
    std::printf("all SimPlantModel tests passed\n");
    return 0;
  }
  std::printf("%d SimPlantModel check(s) FAILED\n", failures);
  return 1;
}
