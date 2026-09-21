---
name: add-hardware
description: Add a new ros2_control hardware plugin (sensor or actuator) or a new controller to ROCKO. Use when adding a sensor, motor, encoder, IMU, or servo, writing a SystemInterface/SensorInterface/ActuatorInterface, creating a chainable controller, or when a new plugin is not being found at launch ("class not found", pluginlib errors).
---

# Adding a hardware plugin or controller

## Teach the layering first

Before writing code, make sure the human can answer — and if they can't, explain it:

- **Why does `ros2_control` split hardware from controllers?** So "how do I talk to this motor" is
  swappable without touching "what should the motor do." That is what lets the same cascade run
  against a real motor, a mock, or a simulator.
- **Which interface does this device need?** `SensorInterface` (reads only — IMU, encoder),
  `ActuatorInterface` (one commanded joint — a motor), `SystemInterface` (several, coupled).
- **Are you exporting state interfaces, command interfaces, or both?** And what *names* — the names
  are the contract the YAML wires against, and a typo here is the single most common cause of a
  controller stuck `inactive`.

Get their answers, present your plan, then implement.

## Start from the templates

`examples/example_hardware/` has the skeletons: `hardware_template.hpp` / `.cpp` for C++ and
`hardware_template.py` for Python. Copy, don't write from scratch.

**New device drivers belong in C++**, not Python. They are called from the 100 Hz loop, and a
Python node reached over DDS cannot meet that budget — see the `control-loop-change` skill.

## Wiring checklist — all five, or the plugin is invisible

A new plugin that compiles and still "isn't found" has missed one of these.

1. **Source file** under `rocko_env/rocko_env/hardware/<kind>/<name>/` (or
   `.../controllers/<name>/`), with headers in a sibling `include/rocko_env/`.
2. **`rocko_env/CMakeLists.txt`** — four separate places:
   - add the `.cpp` to `add_library(rocko_env SHARED ...)` (~line 45)
   - add the include dir to `target_include_directories` (~line 64)
   - add the include dir to the first `install(DIRECTORY ...)` block (~line 88)
   - if it's a Python executable, add it to `install(PROGRAMS ...)` (~line 115)
3. **Plugin XML** — `rocko_env_hardware.xml` for hardware, `rocko_env_controllers.xml` for
   controllers. `name`, `type`, and `base_class_type` must all be right, plus a real `<description>`:
   ```xml
   <class name="rocko_env/MyDevice"
          type="rocko_env::MyDevice"
          base_class_type="hardware_interface::SensorInterface">
     <description>What it is, in a sentence a student can use.</description>
   </class>
   ```
4. **`package.xml`** — add any new dependency. `rosdep` installs from here; skipping it breaks
   everyone else's build, not yours.
5. **URDF / ros2_control xacro** — `rocko_env/rocko_env/description/ros2_control/*.xacro` says which
   plugin drives which joint and carries the per-device params (GPIO pins, I²C address). A new
   device is unused until it appears here.

If the controller needs parameters, use `generate_parameter_library` the way
`add_feedforward_controller_parameters.yaml` does, rather than hand-rolled parameter reads.

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

1. Build — see the `build-verify` skill. Off-Pi, anything touching WiringPi will not link; say so.
2. `xacro` the URDF and `check_urdf` the output — catches wiring errors with no hardware present.
3. On the Pi: `ros2 control list_hardware_interfaces` should show your new interface names exactly
   as the YAML expects them.
4. Run the review loop (`CLAUDE.md` §5) with `ros2-control-reviewer`, and `realtime-auditor` too if
   the device sits in the control path.

## Document it

A new device is not done until:
- its wiring (pins, bus address, voltage) is in `docs/architecture/`,
- the interface names it exports are written down where the next person will look,
- and the README's folder map still matches reality.
