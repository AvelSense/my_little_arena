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
CarController::CarController()
: controller_interface::ControllerInterface(), subscriber_command_(nullptr)
{
}

controller_interface::CallbackReturn CarController::on_init()
{
  try
  {
    declare_parameters();
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
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  subscriber_command_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), 
      [this](const CmdType::SharedPtr msg){callback_command(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CarController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;
  return command_interfaces_config;
}

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
  // Reset commands
  set_actuator_commands(std::vector<double>(command_interface_names_.size(), 0.0));
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands
  set_actuator_commands(std::vector<double>(command_interface_names_.size(), 0.0));
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type CarController::update( // Main entrypoint of the controller, called in the control loop
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto target_velocity = commands_in_.linear.x;
  auto target_steering_angle = commands_in_.angular.z;
  auto states_raw = get_states();

  double propulsion_torque = 0.0;
  double steering_left_position = 0.0;
  double steering_right_position = 0.0;

  set_actuator_commands({steering_left_position, 
    steering_right_position, 
    propulsion_torque, 
    propulsion_torque});

  return controller_interface::return_type::OK;
}


void CarController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn CarController::read_parameters()
{
  params_ = param_listener_->get_params();
  state_interface_names_.push_back(params_.steering_left + "/position");
  state_interface_names_.push_back(params_.steering_right + "/position");
  state_interface_names_.push_back(params_.propulsion_left + "/position");
  state_interface_names_.push_back(params_.propulsion_right + "/position");
  command_interface_names_.push_back(params_.steering_left + "/position");
  command_interface_names_.push_back(params_.steering_right + "/position");
  command_interface_names_.push_back(params_.propulsion_left + "/effort");
  command_interface_names_.push_back(params_.propulsion_right + "/effort");

  return controller_interface::CallbackReturn::SUCCESS;
}


void CarController::callback_command(const CmdType::SharedPtr msg)
{
  commands_in_ = *msg;
}

void CarController::set_actuator_commands(const std::vector<double> & commands)
{
  commands_out_ = commands;
  for (const auto& [interface, command] : std::views::zip(command_interfaces_, commands))
  {
    bool ret = interface.set_value(command);
    if (!ret)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command for interface: %s", interface.get_name().c_str());
    }
  }
}

std::vector<double> CarController::get_states()
{
  std::vector<double> states;
  for (const auto& interface : state_interfaces_)
  {
    // TODO: error management if value is not available
    std::optional<double> value = interface.get_optional();
    states.push_back(value.value_or(0.0));
  }
  return states;
}

}  // namespace simple_car_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  simple_car_controller::CarController,
  controller_interface::ControllerInterface)