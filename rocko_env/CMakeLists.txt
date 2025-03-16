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
  # Hardware .cpp files
  rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp
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
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/controllers/add_feedforward/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rocko_env/controllers/ratelimiting>
$<INSTALL_INTERFACE:include/rocko_env>
)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(rocko_env PUBLIC 
                        Python::Python 
                        ${WIRINGPI_LIBRARIES} 
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

install(PROGRAMS
  rocko_env/hardware/sensors/icm20948/ICM20948.py
  rocko_env/hardware/sensors/quadEncoder/QuadEncoder.py
  rocko_env/controllers/balancing/BalancingController.py
  rocko_env/controllers/diffdrive/DiffDriveController.py
  rocko_env/controllers/ratelimiting/RateLimitingController.py
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_rocko_env HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

