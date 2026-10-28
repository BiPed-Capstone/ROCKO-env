cmake_minimum_required(VERSION 3.16)
project(rocko_env LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
  rocko_interfaces
  generate_parameter_library
  parameter_traits
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. ")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python REQUIRED Development)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(add_feedforward_controller_parameters
  rocko_env/controllers/add_feedforward/add_feedforward_controller_parameters.yaml
)

## COMPILE
add_library(
  rocko_env
  SHARED
  # Hardware .cpp files (the WiringPi motor plugin is added below, once we know WiringPi exists)
  rocko_env/hardware/sensors/icm20948/ICM20948.cpp
  rocko_env/hardware/WiringPi/wiringPi.h
  rocko_env/hardware/sensors/quadEncoder/QuadEncoder.cpp
  # Controller .cpp files
  rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp
)

target_compile_features(rocko_env PUBLIC cxx_std_17)

ament_target_dependencies(
  rocko_env PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(rocko_env PUBLIC
# paths to hardware include folders
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/hardware/motors/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/hardware/sensors/icm20948/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/hardware/sensors/quadEncoder/include>
# Paths to controller include folders
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/controllers/joystick/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/controllers/add_feedforward/include>
$<INSTALL_INTERFACE:include/rocko_env>
)

# WiringPi drives the Pi's GPIO pins, and only the motor plugin uses it. It exists only on a
# Raspberry Pi, so a laptop/container build has to opt out explicitly:
#   colcon build --cmake-args -DROCKO_SKIP_WIRINGPI=ON
# The default is deliberately unchanged: with WiringPi missing the build FAILS, so a robot can
# never be built without its motor driver by accident. A skip build loads and wires the
# controllers (with use_mock_hardware:=true), but it cannot drive motors.
# See docs/decisions/0003-simulation-target.md.
option(ROCKO_SKIP_WIRINGPI "Leave out the WiringPi motor plugin (off-Pi builds only)" OFF)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

if(ROCKO_SKIP_WIRINGPI)
  if(WIRINGPI_LIBRARIES)
    # CMake remembers an option in build/<package>/CMakeCache.txt between builds, so a skip build
    # run once in a Pi's build directory would keep skipping. WiringPi being present here is the
    # giveaway. A warning would print once and then be silent on every later build, so this stops
    # the build instead: a robot must never be built without its motor driver by accident.
    message(FATAL_ERROR "ROCKO_SKIP_WIRINGPI=ON, but WiringPi IS installed here. That is almost "
                        "certainly a stale CMake cache. On a Pi run `rm -rf build/rocko_env` (or "
                        "pass -DROCKO_SKIP_WIRINGPI=OFF) and build again.")
  endif()
  message(WARNING "ROCKO_SKIP_WIRINGPI=ON: building WITHOUT Motor12VoltQuadEncoder. "
                  "This build cannot drive motors; use it only with use_mock_hardware:=true.")
else()
  if(NOT WIRINGPI_LIBRARIES)
    message(FATAL_ERROR "WiringPi not found. On a Raspberry Pi, install it. On a laptop or in "
                        "the Docker container, pass -DROCKO_SKIP_WIRINGPI=ON (no motor plugin).")
  endif()
  target_sources(rocko_env PRIVATE rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp)
  target_link_libraries(rocko_env PUBLIC ${WIRINGPI_LIBRARIES})
endif()

target_link_libraries(rocko_env PUBLIC
                        Python::Python
                        add_feedforward_controller_parameters)

# Export plugins
pluginlib_export_plugin_description_file(hardware_interface rocko_env_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface rocko_env_controllers.xml)

# INSTALL
# HARDWARE INSTALL BLOCK BELOW
install(
  DIRECTORY rocko_env/hardware/motors/include/ 
            rocko_env/hardware/sensors/icm20948/include/ 
            rocko_env/hardware/sensors/quadEncoder/include/ 
            rocko_env/controllers/add_feedforward/include/
  DESTINATION include
)
install(
  DIRECTORY rocko_env/description/launch 
            rocko_env/description/ros2_control 
            rocko_env/description/urdf
  DESTINATION share/rocko_env
)
install(
  DIRECTORY rocko_env/bringup/launch 
            rocko_env/bringup/config
  DESTINATION share/rocko_env
)
install(TARGETS rocko_env add_feedforward_controller_parameters
  EXPORT export_rocko_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Python stuff
ament_python_install_package(${PROJECT_NAME})

# With --symlink-install these are run in place, not copied, so each source file must be
# executable (git mode 100755) and start with a shebang. RateLimiter.py is only imported, not run.
install(PROGRAMS
  rocko_env/hardware/sensors/icm20948/ICM20948.py
  rocko_env/hardware/sensors/quadEncoder/QuadEncoder.py
  rocko_env/hardware/sensors/quadEncoder/QuadEncoders.py
  rocko_env/controllers/diffdrive/DiffDriveController.py
  rocko_env/controllers/utils/RateLimiter.py
  rocko_env/controllers/joystick/Joystick.py
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_rocko_env HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

