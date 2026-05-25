#include "car_controller.hpp"

#include <ranges>

#include "controller_interface/helpers.hpp" 
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace
{  // utility, can only be accessed in this file (same as declaring static functions)


}  // namespace


namespace simple_car_controller
{
CarController::CarController() : controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn CarController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->try_update_params(params_))
  {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  try
  {
    cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

    // Allocate reference interfaces if needed
    const int number_ref_interfaces = 2;
    reference_interfaces_.resize(number_ref_interfaces, std::numeric_limits<double>::quiet_NaN());

    state_interface_names_.push_back(params_.steering_left + '/' + HW_POSITION);
    state_interface_names_.push_back(params_.steering_right + '/' + HW_POSITION);
    state_interface_names_.push_back(params_.propulsion_left + '/' + HW_POSITION);
    state_interface_names_.push_back(params_.propulsion_right + '/' + HW_POSITION);

    command_interface_names_.push_back(params_.steering_left + '/' + HW_POSITION);
    command_interface_names_.push_back(params_.steering_right + '/' + HW_POSITION);
    command_interface_names_.push_back(params_.propulsion_left + '/' + HW_EFFORT);
    command_interface_names_.push_back(params_.propulsion_right + '/' + HW_EFFORT);

    subscriber_command_ = get_node()->create_subscription<CmdType>(
        "~/commands", rclcpp::SystemDefaultsQoS(), 
        [this](const CmdType::SharedPtr msg){callback_command(msg);});

    // initialize odometry publisher and message
    publisher_control_output_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/controller_output", rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
        publisher_control_output_);

    previous_update_timestamp_ = get_node()->get_clock()->now();

  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(logger, "Failed to configure controller: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Declare the interfaces that this controller will export to other controllers in the chain
std::vector<hardware_interface::CommandInterface>
CarController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), "velocity",
      &reference_interfaces_[num_ref_interfaces::VELOCITY]));

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), "turning_radius",
      &reference_interfaces_[num_ref_interfaces::TURNING_RADIUS]));

  return reference_interfaces;
}

// Declare the interfaces that this controller will claim to write commands
controller_interface::InterfaceConfiguration
CarController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;
  return command_interfaces_config;
}

// Declare the interfaces that this controller will access to read states
controller_interface::InterfaceConfiguration 
CarController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;
  return state_interfaces_config;
}

controller_interface::CallbackReturn CarController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  steering_left_.feedback_position = state_interfaces_[num_hardware_interfaces::STEERING_LEFT];
  steering_left_.command_position = command_interfaces_[num_hardware_interfaces::STEERING_LEFT];

  steering_right_.feedback_position = state_interfaces_[num_hardware_interfaces::STEERING_RIGHT];
  steering_right_.command_position = command_interfaces_[num_hardware_interfaces::STEERING_RIGHT];

  propulsion_left_.feedback_position = state_interfaces_[num_hardware_interfaces::PROPULSION_LEFT];
  propulsion_left_.command_effort = command_interfaces_[num_hardware_interfaces::PROPULSION_LEFT];
  
  propulsion_right_.feedback_position = state_interfaces_[num_hardware_interfaces::PROPULSION_RIGHT];
  propulsion_right_.command_effort = command_interfaces_[num_hardware_interfaces::PROPULSION_RIGHT];

  // Reset commands
  halt();

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands
  halt();
  reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CarController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();

  auto rt_box_status = rt_box_commands_in_.try_get();
  if (rt_box_status.has_value())
  {
    commands_in_ = rt_box_status.value();
  }

  if (!commands_in_.has_value() || time - commands_in_.value().header.stamp > cmd_vel_timeout_)
  {
    halt(); 
    return controller_interface::return_type::OK;
  }
  else{
    reference_interfaces_[0] = commands_in_.value().twist.linear.x;
    reference_interfaces_[1] = commands_in_.value().twist.angular.z;
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type DiffDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  double target_velocity = reference_interfaces_[0];
  double target_turning_radius = reference_interfaces_[1];

  if (!std::isfinite(target_velocity) || !std::isfinite(target_turning_radius))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  std::vector<double> states;
  for (const auto & wheel : {steering_left_, steering_right_, propulsion_left_, propulsion_right_})
  {
    interface = wheel.feedback_position.get();
    auto state_op = interface.get_optional();
    if (!state_op.has_value())
    {
      RCLCPP_DEBUG(logger, "Unable to retrieve the data from the feedback interface %s!", wheel.feedback_position.get().get_name());
      return controller_interface::return_type::OK;
    }
    states.push_back(state_op.value());
  }

  // Set wheels velocities:
  bool set_command_result = true;
  for (size_t index = 0; index < static_cast<size_t>(wheels_per_side_); ++index)
  {
    set_command_result &=
      registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    set_command_result &=
      registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  }

  RCLCPP_DEBUG_EXPRESSION(
    logger, !set_command_result, "Unable to set the command to one of the command handles!");

  return controller_interface::return_type::OK;
}

void CarController::callback_command(const CmdType::SharedPtr msg)
{
  const auto current_time_diff = get_node()->now() - msg->header.stamp;
  if (
    cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
    current_time_diff < cmd_vel_timeout_)
  {
    rt_box_commands_in_.set(*msg);
  }
  else
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Ignoring the received message (timestamp %.10f) because it is older than "
      "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
      rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
      cmd_vel_timeout_.seconds());
  }
}

void CarController::halt(){
  // Reset commands
  reference_interfaces_[0] = 0.0;
  reference_interfaces_[1] = 0.0;
  steering_left_.command_position.get().set_value(0.0);
  steering_right_.command_position.get().set_value(0.0);
  propulsion_left_.command_effort.get().set_value(0.0);
  propulsion_right_.command_effort.get().set_value(0.0);
}

void CarController::reset(){
  halt();
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  commands_in_.reset();
  rt_box_commands_in_.set(commands_in_);
}

}  // namespace simple_car_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  simple_car_controller::CarController,
  controller_interface::ChainableControllerInterface)