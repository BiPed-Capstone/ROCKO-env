# Measure the robot: mass, centre of mass, inertia, wheel radius, motor constants, IMU delay, encoder scale

**Who this is for:** a person with the robot and a bench. An AI agent cannot do any of this: it has
no robot. The agent's part comes afterwards (see "What happens to your numbers").

**Why it exists:** every physical number in the simulated robot (`SimPlant`) is an **estimate, not
measured** ([sim-plant.md §4.1](../architecture/sim-plant.md#41-physical-parameters-estimate-not-measured),
[the gap page](../architecture/sim-to-real-gap.md)). Until these are measured, a gain that
"balances in simulation" says nothing about the robot. Requested by
[proposal 0007](../proposals/0007-measure-robot-parameters.md).

**Machines:** the bench steps need no computer. Steps 6 to 8 use the Raspberry Pi (the robot's
computer); every Pi terminal starts with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Safety, once:** steps 1 to 5 are done with the **battery disconnected** and the motors
unpowered. Step 6 powers a motor from a bench supply; steps 7 and 8 run the robot's software on a
stand with both wheels clear of everything. Keep a way to cut motor power within reach.

## The record to fill in for every measurement

Copy this block from [reproducing-results.md](reproducing-results.md) once per measurement. Every
field, or the word `unknown`.

```
Result:        <the number, with unit, and how many repeats>
Date:          YYYY-MM-DD
Commit:        <git rev-parse --short HEAD on the Pi, and whether the tree was dirty>
Launch:        <launch file and arguments, or "none (bench)">
Hardware:      real
Config:        <which YAML, any gain that differs from the committed file, or "n/a">
Battery:       <volts at start, volts at end, or "n/a (bench supply at ... V)">
Surface:       <what the robot or part sat on>
Robot:         <which physical unit>
Payload:       <anything added or removed, e.g. "wheels and belts removed">
Logs:          <where the photos, video, bag or notebook page lives>
Observed:      <the raw readings, before any formula>
```

Write down **raw readings** first (scale readings, stopwatch times, distances), then the formula.
Repeat each measurement at least three times and record the spread; one reading is not a
measurement.

## Where each number goes

| Step | Number | Plugin / xacro name (`rocko.urdf.xacro`) | Also |
| --- | --- | --- | --- |
| 1 | wheel mass, each | `wheel_mass_kg` | |
| 1 | body mass | `body_mass_kg` | |
| 2 | axle-to-centre-of-mass distance | `com_height_m` | |
| 3 | body pitch inertia about its centre of mass | **no parameter yet** (the model derives it from `body_height_m` and `body_depth_m`) | see step 3 |
| 4 | wheel inertia | **no parameter** (derived, solid disc) | record it; needs a code change to use |
| 5 | wheel radius | `wheel_radius_m` | [issue #16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16), next-steps row 0.7 |
| 5 | belt ratio | `belt_ratio` | |
| 6 | stall torque, no-load speed, the voltage they were measured at, stall and no-load current | `motor_stall_torque_Nm`, `motor_no_load_speed_rad_s`, `motor_rated_voltage_V`, `motor_stall_current_A`, `motor_no_load_current_A` | |
| 6 | battery voltage during the test | `battery_voltage_V` | |
| 7 | IMU delay | `imu_delay_ms` | |
| 8 | encoder metres per wheel turn, counts per turn | no parameter | the gap page, section 3.9 |

The defaults live in three places that `sim/check_sim_config.py` keeps equal
(`rocko.urdf.xacro`, `SimPlantModel.hpp`, and for four of them `rocko_sim.launch.py`): changing a
default is the agent's job, in one commit, not yours.

---

## 1. Mass

**Tools:** a kitchen or postal scale (resolution 1 g or better).

1. Disconnect the battery. Weigh the **whole robot** with the battery fitted. Repeat 3 times.
2. Remove each wheel with its tyre, hub and belt pulley. Weigh each wheel assembly separately,
   3 times. Decide where the belts count and write it in `Payload:`.
3. Weigh the remainder (the body: frame, motors, battery, Pi, boards, wiring).

**Record:** `wheel_mass_kg` (one wheel assembly, in kg; left and right if they differ),
`body_mass_kg` (the remainder in kg). Check: body + 2 wheels equals the whole-robot weight within
a few grams.

## 2. Centre-of-mass height `com_height_m`

The model wants the distance from the **axle** to the **body's** centre of mass, along the body's
long axis, for the body without the wheels.

**Tools:** two scales, a rigid flat board, two small blocks to make both support points the same
height, a ruler.

1. With the wheels removed, lay the body on its back (axle line horizontal and across the
   board) so the body's long axis is horizontal.
2. Put one support under the **axle line** (the line through the two motor shafts). Put the second
   support under the far end of the body, with the body level. Let each support rest on a scale.
3. Read the scale under the far end, `m_B` [kg], and measure the distance `L` [m] between the two
   supports along the body axis.
4. `com_height_m = m_B × L / body_mass_kg`. (Moments about the axle support.)
5. Repeat 3 times, re-seating the body each time.

**Record:** `com_height_m` [m]. Also record `L`, `m_B`, and `m_A` (the other scale): `m_A + m_B`
must equal `body_mass_kg`.

## 3. Body pitch inertia

The pitch axis is the axle line. The model needs the body's inertia about the **parallel axis
through its own centre of mass**, `I_b` [kg·m²].

**Tools:** two strings or thin cords of equal length (about 1 m), a rigid frame or ceiling hook
to hang them from, a stopwatch (phone is fine), a ruler.

**Method A, bifilar swing (primary):** hang the body, wheels removed, from two parallel strings
with the **axle line vertical** (the body lying on its side, hanging level) so that the rotation
you measure is a rotation about a line parallel to the axle.

1. Attach the two strings to the body at two points an equal distance `a` [m] either side of the
   centre of mass (use step 2's result to find the centre of mass; a light rigid bar bolted to the
   frame is fine, weigh it and measure its swing alone in step 4 below to subtract it).
2. Let the body hang still and level. String length `Ls` [m], from the hook to the body.
3. Twist the body a few degrees (under 10) about the vertical and release. Time 20 full
   oscillations; `T` [s] = time / 20. Repeat 3 times.
4. Repeat with the fixture alone (the bar, no body) to get its own inertia, if you used one.
5. `I = m × g × a² × T² / (4π² × Ls)`, with `m` the total hanging mass [kg], `g = 9.81`
   m/s²; subtract the fixture's value for `I_b`.

**Method B, physical pendulum (cross-check):** hang the body (wheels removed) from a rod through
the motor-shaft positions (the axle line). Time 20 small swings (under 10 degrees); `T` [s]. Then
`I_pivot = m × g × l × T² / (4π²)` with `l` = `com_height_m` from step 2, and
`I_b = I_pivot − m × l²`. The two methods should agree within about 10 %; if they do not, write
the two numbers down and say so.

**Record:** `I_b` [kg·m²] from method A, and from method B. **Where it goes:** the model has no
inertia parameter, only `body_height_m` and `body_depth_m` of an equivalent box,
`I_b = m_b (h² + d²) / 12` ([sim-plant.md §4.1](../architecture/sim-plant.md)). The agent will
either add a `body_inertia_kg_m2` parameter (a code change that touches the xacro, the header and
the launch file together) or pick an equivalent `body_height_m` and say it is not a real
dimension. That is the agent's choice, written in the journal.

## 4. Wheel inertia (optional)

Repeat step 3, method A, with one wheel assembly hung flat (axle vertical) from the two strings.
**Record** `I_w` per wheel [kg·m²]. The model uses a solid disc, `½ m r²`, and has no parameter for
it, so this number cannot be used without a code change; record it so the disc assumption can be
judged.

## 5. Wheel radius, belt ratio

This step settles [issue #16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16): the repo
holds four values (0.06, an effective 0.0813, 0.127 and 0.015 m;
[the gap page](../architecture/sim-to-real-gap.md#5-the-wheel-radius-four-values-disagree-issue-16-none-is-picked-here)).
**Do not pick one of them. Measure.**

**Tools:** calipers or a tape, a marker, a long tape measure.

1. Unloaded diameter: measure the outside diameter of the tyre at three places; radius = mean / 2.
2. **Rolling radius (the one that matters):** stand the robot on a flat floor, unpowered, upright
   and held. Mark a tyre at the floor contact and mark the floor. Roll the robot straight until the
   mark has touched the floor `N` times (use `N = 5` or more). Measure the distance `D` [m] between
   the first and last floor marks. `r_rolling = D / (2π × N)`.
3. Repeat 3 times, and for both wheels.
4. Belt ratio: count the teeth on the motor-side pulley and the wheel-side pulley (or the pulley
   diameters if there are no teeth) and write both numbers down. `belt_ratio` = the ratio by which
   the wheel turns slower than the gearbox output (the code's comment says 1.355; not verified).

**Record:** `r_unloaded`, `r_rolling` [m], both pulley tooth counts, `belt_ratio`. Say which one
you think the model should use; the agent does not pick. **Where it goes:** `wheel_radius_m` and
`belt_ratio`; also the row for issue #16 / next-steps row 0.7. Several places in the code hold
their own copy of the radius (`DiffDriveController.py`, `QuadEncoders.py`, the tank-mode YAML,
the URDF): replacing them is a separate, human-approved change.

## 6. Motor constants: stall torque, no-load speed

The model needs, **at the gearbox output shaft** (before the belt), the stall torque and the
no-load speed **at a stated voltage**. `SimPlant` takes that voltage as `motor_rated_voltage_V`,
so you may measure at a safe low voltage and need no scaling.

**Tools:** a bench power supply with a current limit and a voltage and current display, a lever
arm (a rigid bar clamped to the output shaft), a scale, a clamp for the motor, a tachometer (an
optical one, or a phone filming a mark on the shaft at 240 frames per second).

**No-load speed, current:**

1. Clamp the motor, nothing attached to the shaft. Set the supply to a voltage `V_test` [V] (6 V is
   a reasonable start; write down what you used).
2. Run it. Read `I_nl` [A] from the display once the speed settles. Measure the shaft speed (count
   marks over 10 s with a tachometer, or count video frames per turn): `ω_noload` [rad/s] =
   2π × rev/s.

**Stall torque, current (short runs):**

1. Fit the lever arm to the output shaft; the other end presses on the scale, which sits so the
   arm is horizontal. Arm length from the shaft centre to the scale contact: `L_arm` [m].
2. Set the supply to `V_test` with the current limit set to a value below the motor's stall current
   (write the limit down). Switch on for **at most 2 seconds**, read the scale in kg (`F = reading ×
   9.81` N) and the current `I_stall` [A], switch off, and let the motor cool for a minute between
   runs. 3 runs.
3. `τ_stall = F × L_arm` [N·m]. If the current limit stopped the motor from reaching true stall
   current, `τ_stall` is lower than the motor's real stall torque: write the limit down and say so.

**Record:** `τ_stall` [N·m], `ω_noload` [rad/s], `I_stall`, `I_nl` [A], and `V_test` [V]. **Where
it goes:** `motor_stall_torque_Nm`, `motor_no_load_speed_rad_s`, `motor_stall_current_A`,
`motor_no_load_current_A`, and `motor_rated_voltage_V` = `V_test`. Separately record the **battery**
voltage at rest (multimeter across the pack, 3 readings) as `battery_voltage_V`, and its value
after a minute of load if you can: the model holds it constant, the robot's falls (gap page,
section 3.3).

## 7. IMU delay

`imu_delay_ms` stands for how long after the robot moves the pitch reaches software. This
measurement needs the robot's IMU, so it is an **on-robot step**, but it needs no controllers and
no motor power: the IMU node runs alone, the robot sits on a stand, and nothing can drive a wheel.

**Why tank mode will not do:** tank mode (`rocko_tank_mode.launch.py`) has no balance loop, and
`rocko.launch.py` is the launch that starts the IMU node (`rocko.launch.py:110`, only when
`use_mock_hardware` is false). So this procedure runs the node by itself, as
[proposal 0008, section A](../proposals/0008-candidate-gains-and-tethered-test.md) does.

**What the data is, honestly:** the node answers a service, `/icm20948_data`. The reply
(`rocko_interfaces/srv/Icm20948Data`) holds yaw, pitch, roll and `success`, and **has no timestamp**.
So there is no sensor clock to compare against: the only time a reading can have is the moment a
client receives it. This procedure therefore measures *physical motion to a reply reaching a
command-line client* on the Pi, which includes the sensor's own lag, the service round trip and the
client's printing. It does not include the 100 Hz control cycle or the C++ plugin that the real loop
adds on top, so it is a **lower bound** on the delay the controllers see, not the delay itself.

**Tools:** the Pi with a display (or a screen showing an SSH terminal), a phone that films at 60
frames per second if it can (30 if not).

1. On the Pi, in one terminal:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 run rocko_env ICM20948.py
   ```

2. In a second Pi terminal, call the service repeatedly and print each reply as it arrives.
   `ros2 service call` has a `-r` (repeat rate, Hz) option in recent ROS 2 releases; check that
   `ros2 service call --help` lists it on your Pi first. If it does not, wrap the call in a shell
   `while true` loop (slower, because each call starts a new process). **Not run by the author of
   this guide: the command is a composition of standard tools, not a tested script.**

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 service call -r 20 /icm20948_data rocko_interfaces/srv/Icm20948Data
   ```

3. Put the robot's body and that terminal in **one** camera frame. Film at 60 fps (30 if that is all
   you have). Tilt the body by hand, quickly and sharply, about 10 degrees, several times.
4. Play the video frame by frame. For each tilt, note the frame where the body first visibly
   moves, and the first frame where the terminal shows a `pitch` that has clearly changed. The
   number of frames between them, times the frame time (16.7 ms at 60 fps, 33.3 ms at 30 fps), is
   one delay sample. No clock needs to match anything, because both events are in the same video.
5. 10 tilts; record the mean and the range.

**Its uncertainty:** each of the two events is read to within one video frame, and the terminal
adds its own redraw time (one display refresh, about 17 ms, plus whatever the terminal program
adds). A sample is good to about **±20 ms at 60 fps and ±35 ms at 30 fps**, and averaging 10 tilts
does not remove the part that is the same every time (the display and the client). The `-r` rate
also limits it: the sensor is only read when a call arrives, so at 20 Hz a tilt waits up to 50 ms
for the next call and a sample can read up to 50 ms long. Use a higher rate (`-r 50`) if the Pi
keeps up. Say all of this when you record the value. If you cannot
do this, write `not measured`; do not substitute a datasheet number.

Also useful, since the loop's timing matters (finding H-5): with a full launch running (the balancing launch, or the tank-mode one of step 8; not the lone IMU
node above, which publishes no state), in another terminal:

```bash
ros2 topic hz /dynamic_joint_states
```

and write down the reported rate and its spread (it is the update rate of the state, not a
histogram of the loop; the findings ask for a histogram).

**Record:** `imu_delay_ms` [integer ms], the range, how many tilts. **Where it goes:**
`imu_delay_ms`.

## 8. Encoder scale: metres per wheel turn, counts per turn

The encoder node reports metres by dividing counts by a constant built from three comments in
`QuadEncoders.py` (145 pulses per revolution at the gearbox output, belt 1.355, radius 0.06 m, which
is 521.2 counts per metre). If those are wrong the velocity the loops see is wrong by the same
factor, and so is the velocity loop's gain (gap page, section 3.9).

**Tools:** a marker, the Pi.

1. Robot on a stand, motors unpowered (or the launch below with no command), wheels free.
2. On the Pi:

   ```bash
   ros2 launch rocko_env rocko_tank_mode.launch.py
   ```

3. In a second terminal read the encoder position (metres, **left wheel**), before and after
   turning the **left wheel by hand** exactly `N` full turns (mark the tyre; `N = 10`):

   ```bash
   ros2 service call /encoder_data rocko_interfaces/srv/QuadEncoderData
   ```

   (`ros2 service list -t` shows the type if it differs. Each call also moves the node's "previous
   position", which only disturbs its velocity reading, not the position you want.) Repeat 3 times.
4. `reported_m_per_turn = (position_after − position_before) / N`. Compare with the circumference
   from step 5, `2π × r_rolling`.
5. `counts_per_wheel_turn = reported_m_per_turn × 521.2` (the code's counts per metre; this is the
   number of counts the node believes it saw per turn, so it is exact only if the code's constant
   is the one in use: check the file).

**Record:** `reported_m_per_turn`, `2π × r_rolling` [m], and the ratio of the two (1.00 means the
node's metres are true; the code's comments predict 196.5 counts per turn, unverified). **Where it
goes:** no `SimPlant` parameter; the ratio goes to the gap page (section 3.9) and to whichever human
decision fixes the constant.

---

## What happens to your numbers

1. Put the records somewhere durable: a new file `docs/journal/YYYY-MM-DD-measure-robot.md` (the
   template is in [the journal README](../journal/README.md)) or a pull request description.
2. Tell the agent. It will, in one commit: update the `ESTIMATE, NOT MEASURED` table in
   [sim-plant.md §4.1](../architecture/sim-plant.md) and its copy in
   [the gap page](../architecture/sim-to-real-gap.md) (an estimate becomes "measured on date, by
   method"), change the three default sites together (and `sim/check_sim_config.py` must stay
   green), then **rerun the tuning scenarios and the ±30 % sweep** with the new model. Gains found
   before are then out of date.
3. The wheel radius is also yours to decide for the rest of the code (issue #16).

## What goes wrong

- A formula that uses a unit of grams or centimetres: convert to kg and m before the formula.
- A pivot or string with friction, or an amplitude of more than 10 degrees: the period is then not
  the small-oscillation period. Redo with a smaller swing.
- A current limit that is too low at stall: the torque is under-measured (step 6 says so).
- A video clock out of the frame or blurred: repeat the tilt.
