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
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
find_package(Python REQUIRED Development)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  rocko_env
  SHARED
  hardware/motors/Motor12VoltQuadEncoder.cpp
  hardware/WiringPi/wiringPi.h
)
target_compile_features(rocko_env PUBLIC cxx_std_17)
target_include_directories(rocko_env PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/motors/include>
$<INSTALL_INTERFACE:include/rocko_env>
)
ament_target_dependencies(
  rocko_env PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(rocko_env PRIVATE Python::Python ${WIRINGPI_LIBRARIES})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rocko_env.xml)

# INSTALL
install(
  DIRECTORY hardware/motors/include/
  DESTINATION include/rocko_env
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/rocko_env
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/rocko_env
)
install(TARGETS rocko_env
  EXPORT export_rocko_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_rocko_env HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
