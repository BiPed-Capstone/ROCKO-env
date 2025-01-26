#ifndef ROCKO_ENV_QuadEncoder
#define ROCKO_ENV_QuadEncoder

#include <memory>
#include <string>
#include <vector>  

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "rocko_interfaces/srv/quad_encoder_data.hpp"

namespace rocko_env
{
class QuadEncoder : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(QuadEncoder);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    

private:
  std::string _prefix = "";

  rclcpp::Client<rocko_interfaces::srv::QuadEncoderData>::SharedPtr _client;
  std::shared_ptr<rclcpp::Node> _node;

  std::string POSITION_KEY = "position";
  std::string VELOCITY_KEY = "velocity";

  double _position, _velocity = 0;
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_QuadEncoder