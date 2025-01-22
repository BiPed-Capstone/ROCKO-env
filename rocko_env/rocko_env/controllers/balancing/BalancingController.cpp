#include "rocko_env/BalancingController.hpp"

namespace rocko_env
{
    controller_interface::CallbackReturn on_init()
    {
        // Set up everything
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration()
    {
        std::vector<std::string> conf_names;
        // for (const auto &joint_name : params_.left_wheel_names)
        // {
        //     conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        // }
        // for (const auto &joint_name : params_.right_wheel_names)
        // {
        //     conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        // }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration state_interface_configuration()
    {
        std::vector<std::string> conf_names;
        // for (const auto &joint_name : params_.left_wheel_names)
        // {
        //     conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        // }
        // for (const auto &joint_name : params_.right_wheel_names)
        // {
        //     conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        // }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Grab the values from the state interfaces

        // Do calculations
        
        // Update the command interfaces with what to do

        return controller_interface::return_type::OK;
    }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  rocko_env::BalancingController, controller_interface::ControllerInterface)