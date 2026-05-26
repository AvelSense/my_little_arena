#include "car_controller.hpp"

#include <ranges>
#include <limits>
#include <cmath>
#include <cfloat>
#include <iostream>

#include "controller_interface/helpers.hpp" 
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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

  // update parameters if they have changed
  if (param_listener_->try_update_params(params_))
  {
    RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
  }

  try
  {
    cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

    // Allocate reference interfaces if needed
    const int number_ref_interfaces = 2;
    reference_interfaces_.resize(number_ref_interfaces, std::numeric_limits<double>::quiet_NaN());

    state_interface_names_.push_back(params_.steering_left + '/' + hardware_interface::HW_IF_POSITION);
    state_interface_names_.push_back(params_.steering_right + '/' + hardware_interface::HW_IF_POSITION);
    state_interface_names_.push_back(params_.propulsion_left + '/' + hardware_interface::HW_IF_POSITION);
    state_interface_names_.push_back(params_.propulsion_right + '/' + hardware_interface::HW_IF_POSITION);

    command_interface_names_.push_back(params_.steering_left + '/' + hardware_interface::HW_IF_POSITION);
    command_interface_names_.push_back(params_.steering_right + '/' + hardware_interface::HW_IF_POSITION);
    command_interface_names_.push_back(params_.propulsion_left + '/' + hardware_interface::HW_IF_EFFORT);
    command_interface_names_.push_back(params_.propulsion_right + '/' + hardware_interface::HW_IF_EFFORT);

    subscriber_command_ = get_node()->create_subscription<CmdType>(
        "~/commands", rclcpp::SystemDefaultsQoS(), 
        [this](const CmdType::SharedPtr msg){callback_command(msg);});

    // initialize odometry publisher and message
    publisher_control_output_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/controller_output", rclcpp::SystemDefaultsQoS());
    rt_publisher_control_output_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
        publisher_control_output_);

    previous_update_timestamp_ = get_node()->get_clock()->now();

  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure controller: %s", e.what());
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
  steering_left_.emplace(state_interfaces_[num_hardware_interfaces::STEERING_LEFT],
  command_interfaces_[num_hardware_interfaces::STEERING_LEFT]);

  steering_right_.emplace(state_interfaces_[num_hardware_interfaces::STEERING_RIGHT],
  command_interfaces_[num_hardware_interfaces::STEERING_RIGHT]);

  propulsion_left_.emplace(state_interfaces_[num_hardware_interfaces::PROPULSION_LEFT],
  command_interfaces_[num_hardware_interfaces::PROPULSION_LEFT]);

  propulsion_right_.emplace(state_interfaces_[num_hardware_interfaces::PROPULSION_RIGHT],
  command_interfaces_[num_hardware_interfaces::PROPULSION_RIGHT]);
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

bool CarController::fetch_commands_in(double &target_velocity, double &target_turning_radius)
{
  target_turning_radius = reference_interfaces_[1];
  target_velocity = reference_interfaces_[0];
  const bool res = (std::isfinite(target_velocity) && std::isfinite(target_turning_radius));
  if (!res){RCLCPP_ERROR(get_node()->get_logger(), "Received invalid commands.");}
  return res;
}

bool CarController::fetch_states(double &steering_left_pos, double &steering_right_pos,
  double &propulsion_left_pos, double &propulsion_right_pos)
{
  std::vector<double> states;
  states.reserve(4);
  const auto wheels = std::tie(steering_left_, steering_right_, propulsion_left_, propulsion_right_);

  bool res = true;
  std::apply([&](auto&&... wheel){([&]{
        const auto & interface = wheel.value().feedback_position.value().get();
        auto feedback = interface.get_optional();
        if (feedback.has_value() && std::isfinite(feedback.value()))
        {
          res = false;
          states.push_back(feedback.value());
        }
        else
        {
          RCLCPP_ERROR(get_node()->get_logger(),
            "Cannot fetch valid feedback from %s", interface.get_prefix_name().c_str());
        }
      }(), ...);},
    wheels);
  steering_left_pos    = states[0];
  steering_right_pos   = states[1];
  propulsion_left_pos  = states[2];
  propulsion_right_pos = states[3];
  return res;
}

bool CarController::write_commands_out(const double &steering_left, const double &steering_right,
  const double &propulsion_left, const double &propulsion_right)
{
  bool set_command_result = true;
  set_command_result &= steering_left_.value().command_position.get().set_value(steering_left);
  set_command_result &= steering_right_.value().command_position.get().set_value(steering_right);
  set_command_result &= propulsion_left_.value().command_effort.get().set_value(propulsion_left);
  set_command_result &= propulsion_right_.value().command_effort.get().set_value(propulsion_right);

  if (!set_command_result)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Unable to set the command to one of the command handles!");
  }
  return set_command_result;
}

controller_interface::return_type CarController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double target_turning_radius, target_velocity;
  if (!fetch_commands_in(target_velocity, target_turning_radius)){return controller_interface::return_type::OK;}

  double steering_left_pos, steering_right_pos, propulsion_left_pos, propulsion_right_pos;
  if (!fetch_states(steering_left_pos, steering_right_pos,propulsion_left_pos, propulsion_right_pos))
  {return controller_interface::return_type::OK;}

  double steering_left = 0., steering_right = 0., propulsion_left = 0., propulsion_right = 0.;

  write_commands_out(steering_left, steering_right, propulsion_left, propulsion_right);
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
  bool set_command_result = true;
  set_command_result &= steering_left_.value().command_position.get().set_value(0.0);
  set_command_result &= steering_right_.value().command_position.get().set_value(0.0);
  set_command_result &= propulsion_left_.value().command_effort.get().set_value(0.0);
  set_command_result &= propulsion_right_.value().command_effort.get().set_value(0.0);
  
  RCLCPP_DEBUG_EXPRESSION(
    get_node()->get_logger(), !set_command_result, "Unable to set the command to one of the command handles!");

}

void CarController::reset(){
  halt();
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  commands_in_.reset();
  rt_box_commands_in_.set(commands_in_);
  steering_left_.reset();
  steering_right_.reset();
  propulsion_left_.reset();
  propulsion_right_.reset();
}

}  // namespace simple_car_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  simple_car_controller::CarController,
  controller_interface::ChainableControllerInterface)