#ifndef ROCKO_ENV_BALANCING_CONTROLLER
#define ROCKO_ENV_BALANCING_CONTROLLER

#include "controller_interface/controller_interface.hpp"

namespace rocko_env
{
    class BalancingController : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;
    };
} // namespace rocko_env

#endif
