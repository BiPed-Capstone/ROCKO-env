---
name: add-hardware
description: Add a new ros2_control hardware plugin (sensor or actuator) or a new controller to ROCKO. Use when adding a sensor, motor, encoder, IMU, or servo, writing a SystemInterface/SensorInterface/ActuatorInterface, creating a chainable controller, or when a new plugin is not being found at launch ("class not found", pluginlib errors).
---

# Adding a hardware plugin or controller

You may do this end to end without asking. The one thing that must stay true afterwards is that the
**mock still mirrors the real hardware** — see "Keep the mock in step" — because Tier 2 only means
something while it does.

## Decide three things first, and write them in the run journal

A student will later ask why the code is shaped this way; these are the answers.

- **Why does `ros2_control` split hardware from controllers?** So "how do I talk to this motor" is
  swappable without touching "what should the motor do." That is what lets the same cascade run
  against a real motor, a mock, or a simulator.
- **Which interface does this device need?** `SensorInterface` (reads only — IMU, encoder),
  `ActuatorInterface` (one commanded joint — a motor), `SystemInterface` (several, coupled).
- **Which state and command interfaces, and under what names?** The names are the contract the YAML
  wires against, and a typo here is the single most common cause of a controller stuck `inactive`.
  Choose names that describe themselves (`pitch`, not `s2`).

## Start from the templates

`examples/example_hardware/` has the skeletons: `hardware_template.hpp` / `.cpp` for C++ and
`hardware_template.py` for Python. Copy, don't write from scratch.

**New device drivers belong in C++**, not Python. They are called from the 100 Hz loop, and a
Python node reached over DDS cannot meet that budget — see the `control-loop-change` skill.

## Wiring checklist — all five, or the plugin is invisible

A new plugin that compiles and still "isn't found" has missed one of these.

1. **Source file** under `rocko_env/rocko_env/hardware/<kind>/<name>/` (or
   `.../controllers/<name>/`), with headers in a sibling `include/rocko_env/`.
2. **`rocko_env/CMakeLists.txt`** — four separate places (find them by name; line numbers drift):
   - add the `.cpp` to `add_library(rocko_env SHARED ...)`
   - add the include dir to `target_include_directories`
   - add the include dir to the first `install(DIRECTORY ...)` block (the one under `# HARDWARE INSTALL BLOCK`)
   - if it's a Python executable, add it to `install(PROGRAMS ...)`, give the source file a
     shebang, and make it executable in git (`git update-index --chmod=+x <file>`): under
     `--symlink-install` it is run in place

   If the new source needs a library that exists only on the Pi (as WiringPi does), do **not** add it
   to `add_library`. Add it with `target_sources` inside the `ROCKO_SKIP_WIRINGPI` pattern already in
   that file, so the container build still works and fails closed on the Pi.
3. **Plugin XML** — `rocko_env_hardware.xml` for hardware, `rocko_env_controllers.xml` for
   controllers. `name`, `type`, and `base_class_type` must all be right, plus a real `<description>`:
   ```xml
   <class name="rocko_env/MyDevice"
          type="rocko_env::MyDevice"
          base_class_type="hardware_interface::SensorInterface">
     <description>What it is, in a sentence a student can use.</description>
   </class>
   ```
4. **`package.xml`** — add any new dependency, including *runtime* ones the launch or YAML uses (this
   file once omitted `pid_controller`, which the whole cascade needs). `rosdep` installs from here;
   skipping it breaks everyone else's build, not yours.
5. **URDF / ros2_control xacro** — `rocko_env/rocko_env/description/ros2_control/*.xacro` says which
   plugin drives which joint and carries the per-device params (GPIO pins, I²C address). A new
   device is unused until it appears here.

If the controller needs parameters, use `generate_parameter_library` the way
`add_feedforward_controller_parameters.yaml` does, rather than hand-rolled parameter reads.

## Keep the mock in step

`use_mock_hardware:=true` replaces every real `<ros2_control>` block with one `MockRocko` system
that exports the **same joint and interface names**. If you add or rename an interface in the real
block and not in the mock block (in both `rocko.ros2_control.xacro` and
`rocko_tank_mode.ros2_control.xacro`), Tier 2 will pass against a wiring the robot does not have. Edit
both in the same change, and extend `verify_mock.sh` if a new controller or interface should be
checked.

## Lifecycle: put things in the right method

| Method | Runs | Do here |
| --- | --- | --- |
| `on_init` | once | Validate the URDF info, size every buffer |
| `on_configure` | on configure | Open the device, allocate, read calibration |
| `on_activate` | before the loop starts | Zero state, latch a safe initial command |
| `read` / `write` | **every 10 ms** | Bounded work only — see `control-loop-change` |
| `on_deactivate` | on stop | Command the device to a safe state (motors to zero) |
| `on_cleanup` / `on_shutdown` | teardown | Close handles |

Allocation and I/O setup in `read()` is the classic mistake. Keep the loop empty of everything but
the transfer itself.

## Verify

Work up the ladder in the `build-verify` skill and record each tier in the run journal:

1. Tier 1 first — `build_ros.sh` in the container (`verify_urdf.sh` exits 2 without a build). A
   device that needs a Pi-only library is **not compiled there**; say so.
2. Tier 0 — `verify_urdf.sh`. Extend it if your device adds a plugin whose presence it should assert;
   its parity check already fails if the mock stops exporting what the real hardware exports.
3. Tier 2 — `verify_mock.sh` (and `verify_mock.sh tank`). Add your new controller or interface to
   its expected lists so a regression turns it red.

(Exact commands: the `build-verify` skill.)
4. Hardware — you cannot do this. On the Pi a person runs `ros2 control list_hardware_interfaces`
   and checks your interface names match the YAML exactly. Write that down as a procedure.

Then review: `ros2-control-reviewer` always, `realtime-auditor` if the device sits in the control
path, `explainability-reviewer` and `docs-reviewer` always.

## Document it

A new device is not done until:
- its wiring (pins, bus address, voltage) is in `docs/architecture/`,
- the interface names it exports are written down where the next person will look,
- the README's folder map still matches reality,
- and the verification ladder's status table is still true.
