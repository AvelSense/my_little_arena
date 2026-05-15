#include "description_simple_car/PwmServoActuator.hpp"

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

namespace pwm_servo_actuator
{
hardware_interface::CallbackReturn PwmServoActuator::on_init(const hardware_interface::HardwareComponentInterfaceParams & params){
  if (hardware_interface::ActuatorInterface::on_init(params) !=hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.angle_min = std::stof(info_.hardware_parameters["angle_min"]);
  cfg_.angle_max = std::stof(info_.hardware_parameters["angle_max"]);
  cfg_.pwm_min = std::stoi(info_.hardware_parameters["pwm_min"]);
  cfg_.pwm_max = std::stoi(info_.hardware_parameters["pwm_max"]);
  cfg_.pwm_neutral = std::stoi(info_.hardware_parameters["pwm_neutral"]);
  
  servo_.setup(cfg_.angle_min, cfg_.angle_max, cfg_.pwm_min, cfg_.pwm_max) ;

  const hardware_interface::ComponentInfo & joint = info_.joints[0];

  // SimpleWheelDriver has exactly one state and one command interface on each joint
  if (joint.command_interfaces.size() != 1){
    RCLCPP_FATAL(get_logger(),
      "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
    RCLCPP_FATAL(get_logger(),
      "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(), 
      joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
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
      "Joint '%s' have '%s' as state interface. '%s' expected.", joint.name.c_str(),
      joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmServoActuator::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully shut down!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmServoActuator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(rclcpp::get_logger("PwmServoActuator"), "Configuring ...please wait...");
  servo_.write_neutral_pwm();

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

hardware_interface::CallbackReturn PwmServoActuator::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  servo_.deactivate();
  // RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmServoActuator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmServoActuator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  // RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  // RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type PwmServoActuator::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  for (const auto & [name, descr] : joint_state_interfaces_){
    set_state(name, servo_.read_rotation());
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type PwmServoActuator ::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  for (const auto & [name, descr] : joint_command_interfaces_){
    float pos_cmd = get_command(name);
    servo_.set_angle(pos_cmd);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace pwm_servo_actuator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pwm_servo_actuator::PwmServoActuator, hardware_interface::ActuatorInterface)