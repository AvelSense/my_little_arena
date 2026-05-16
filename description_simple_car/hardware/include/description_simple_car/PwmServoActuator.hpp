#ifndef DESCRIPTION_SIMPLE_CAR__SIMPLE_PWM_SERVO_ACTUATOR_HPP_
#define DESCRIPTION_SIMPLE_CAR__SIMPLE_PWM_SERVO_ACTUATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include "std_msgs/msg/string.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include "description_simple_car/pwm_servo.hpp"

namespace pwm_servo_actuator{

class PwmServoActuator : public hardware_interface::ActuatorInterface
{

struct Config {
  float angle_min = 0;
  float angle_max = 0;
  int pwm_min = 0;
  int pwm_max = 0;
  int pwm_neutral = 0;
  float target_angle = 0;
  int current_pwm = 0;
};

public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  PWM_Servo servo_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_publisher_;
  rclcpp::TimerBase::SharedPtr my_timer_;
};

}  // namespace pwm_servo_actuator

#endif  // DESCRIPTION_SIMPLE_CAR__SIMPLE_PWM_SERVO_ACTUATOR_HPP_