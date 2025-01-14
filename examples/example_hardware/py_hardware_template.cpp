// TODO: Replace <ClassName> with your class name
hardware_interface::CallbackReturn <ClassName>::on_init(
    const hardware_interface::HardwareInfo & info)
{
    // Set up connection to client
    _node = rclcpp::Node::make_shared("<ClassName>_client");
    // TODO: Change ServiceType to the type of service in your .py file
    // TODO: Make the string in here match the service name in your .py file
    _client = _node->create_client<ServiceType>("serviceName");
    
    while (!_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type <ClassName>::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // TODO: Change ServiceType to the type of service in your .py file
    auto request = std::make_shared<ServiceType::Request>();

    // TODO: Fill in data for your request

    auto result = _client->async_send_request(request);
    // Wait for the result.
    if ((rclcpp::spin_until_future_complete(_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) && result.valid())
    {
      auto res = result.get();
      // TODO: Grab the response data and put into variables for you to use
      
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service <ClassName>");
    }

    return hardware_interface::return_type::OK;
}