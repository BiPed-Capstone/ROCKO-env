#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rocko_env/Motor12VoltQuadEncoder.hpp"
#include <wiringPi.h>
#include <softPwm.h>

namespace rocko_env
{
  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    // Check that we have pin names for speed and direction
    if (!(info.hardware_parameters.count(PIN_NUMBER_SPEED_KEY) > 0))
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
          PIN_NUMBER_SPEED_KEY.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else if (!(info.hardware_parameters.count(PIN_NUMBER_DIRECTION_KEY) > 0))
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
          PIN_NUMBER_DIRECTION_KEY.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check that we have a motor
    if (info.joints.size() != 1)
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' has %zu joints found. 1 expected.", info.name.c_str(),
          info.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check that each motor has one command and two states
    for (hardware_interface::ComponentInfo joint : info.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' command interface. Expected a '%s' command interface.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' state interface as the first interface. Expected a '%s' state interface.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' state interface as the second interface. Expected a '%s' state interface.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    _speedPin = stoi(info.hardware_parameters.at(PIN_NUMBER_SPEED_KEY));
    _dirPin = stoi(info.hardware_parameters.at(PIN_NUMBER_DIRECTION_KEY));
    std::istringstream(info.hardware_parameters.at(INVERT_KEY)) >> std::boolalpha >> _invert;

    _wheel.setup(info.joints[0].name, 0);

    // Set up pins to have WiringPi numberings
    wiringPiSetup();

    // Set up client to get encoder data
    _prefix = info.joints[0].name;

    // Set up connection to client
    _node = rclcpp::Node::make_shared(_prefix + "_encoder_client");
    std::string s = _prefix + "_encoder_data";
    _client = _node->create_client<rocko_interfaces::srv::QuadEncoderData>(s);

    while (!_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the %s service. Exiting.", s.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service not available, waiting again...", s.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_configure(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    // Set up the speed pin to be PWM and the dir pin to be output
    pinMode(_speedPin, SOFT_PWM_OUTPUT);
    pinMode(_dirPin, OUTPUT);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> Motor12VoltQuadEncoder::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        _wheel.name, hardware_interface::HW_IF_POSITION, &_wheel.pos));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Motor12VoltQuadEncoder::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Set up PWM
    softPwmCreate(_speedPin, 0, 100);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Stop PWM here
    softPwmStop(_speedPin);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Motor12VoltQuadEncoder::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Read the encoders to find speed and pos, and update the wheel object, which gets read when states are exported
    auto request = std::make_shared<rocko_interfaces::srv::QuadEncoderData::Request>();

    auto result = _client->async_send_request(request);
    // Wait for the result.
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      // TODO: Grab the response data and put into variables for you to use
      _wheel.pos = res->position;
      _wheel.vel = res->velocity;
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pos: %6.2f Vel: %5.2f", _wheel.pos, _wheel.vel);
      
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service QuadEncoder");
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Motor12VoltQuadEncoder::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Convert from velocity in m/s to percent of full speed
    double radPerSec = _wheel.cmd; // Multiply by wheel radius bc they add it for some reason and it makes the velocities wrong
    int pwmVal = (std::sqrt(std::abs((radPerSec / MAX_RAD_PER_SEC))) + 0.15) * 100; // _wheel.cmd holds the speed we want to go

    // Set direction
    if (_wheel.cmd >= 0)
    {
      digitalWrite(_dirPin, !_invert);
    }
    else
    {
      digitalWrite(_dirPin, _invert);
    }

    // Set speed if above threshold
    if (pwmVal > 2) {
      softPwmWrite(_speedPin, pwmVal);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rad/sec: %5.2f PercentOut: %5.2f", radPerSec, pwmVal / 100.0);

    return hardware_interface::return_type::OK;
  }

} // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rocko_env::Motor12VoltQuadEncoder, hardware_interface::ActuatorInterface)
