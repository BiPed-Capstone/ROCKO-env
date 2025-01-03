#ifndef ROCKO_ENV_ICM20948
#define ROCKO_ENV_ICM20948

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "rocko_interfaces/srv/icm20948_data.hpp"

namespace rocko_env
{
class ICM20948 : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string _prefix = "";

  rclcpp::Client<rocko_interfaces::srv::Icm20948Data>::SharedPtr _client;
  std::shared_ptr<rclcpp::Node> _node;

  std::string X_STATE_KEY = "x";
  std::string Y_STATE_KEY = "y";
  std::string Z_STATE_KEY = "z";

  double _x, _y, _z = 0;
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_ICM20948