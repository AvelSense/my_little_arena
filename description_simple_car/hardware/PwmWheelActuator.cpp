#include "description_simple_car/PwmWheelActuator.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pwm_wheel_actuator
{
hardware_interface::CallbackReturn PwmWheelActuator::on_init(const hardware_interface::HardwareComponentInterfaceParams & params){
  if (hardware_interface::ActuatorInterface::on_init(params) !=hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.dc_motor_operating_voltage = std::stof(info_.hardware_parameters["dc_motor_operating_voltage"]);
  cfg_.dc_motor_stall_current = std::stof(info_.hardware_parameters["dc_motor_stall_current"]);
  cfg_.dc_motor_no_load_current = std::stof(info_.hardware_parameters["dc_motor_no_load_current"]);
  cfg_.dc_motor_no_load_speed = std::stof(info_.hardware_parameters["dc_motor_no_load_speed"]);
  cfg_.gear_speed_division = std::stof(info_.hardware_parameters["gear_speed_division"]);
  cfg_.gear_speed_division_efficiency = std::stof(info_.hardware_parameters["gear_speed_division_efficiency"]);
  cfg_.enc_counts_per_rev = std::stof(info_.hardware_parameters["enc_counts_per_rev"]);
  
  wheel_.setup(cfg_.enc_counts_per_rev);
  motor_.setup(cfg_.dc_motor_operating_voltage, cfg_.dc_motor_stall_current, 
    cfg_.dc_motor_no_load_current, cfg_.dc_motor_no_load_speed,
    cfg_.gear_speed_division, cfg_.gear_speed_division_efficiency);

  const hardware_interface::ComponentInfo & joint = info_.joints[0];

  // SimpleWheelDriver has exactly one state and one command interface on each joint
  if (joint.command_interfaces.size() != 1){
    RCLCPP_FATAL(get_logger(),
      "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT){
    RCLCPP_FATAL(get_logger(),
      "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(), 
      joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
      joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
      joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmWheelActuator::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully shut down!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmWheelActuator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(rclcpp::get_logger("PwmWheelActuator"), "Configuring ...please wait...");
  motor_.write_neutral_pwm();

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  if (get_node())
  {
    my_publisher_ = get_node()->create_publisher<std_msgs::msg::String>("simple_wheel_driver_status", 10);

    using namespace std::chrono_literals;
    my_timer_ = get_node()->create_wall_timer(1s, [this]() {
        std_msgs::msg::String msg;
        msg.data = "Hardware status update!";
        my_publisher_->publish(msg);
    });
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmWheelActuator::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  motor_.deactivate();
  // RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmWheelActuator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmWheelActuator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type PwmWheelActuator::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
  for (const auto & [name, descr] : joint_state_interfaces_){
    wheel_.simulate_step(period);
    wheel_.update_measurement(period);
    set_state(name, wheel_.read_rotation());
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type PwmWheelActuator ::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  for (const auto & [name, descr] : joint_command_interfaces_){
    float torque_cmd = get_command(name);
    motor_.apply_torque(torque_cmd, wheel_.read_velocity());
    wheel_.set_simulated_torque(torque_cmd);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace pwm_wheel_actuator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pwm_wheel_actuator::PwmWheelActuator, hardware_interface::ActuatorInterface)