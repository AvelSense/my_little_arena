#ifndef DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_
#define DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_

#include "simple_car_controller_parameters.hpp"

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace simple_car_controller
{
enum order_interfaces
{
    STEERING_LEFT,
    STEERING_RIGHT,
    PROPULSION_LEFT,
    PROPULSION_RIGHT,
};

class CarController : public controller_interface::ControllerInterface
{
using CmdType = geometry_msgs::msg::Twist;
public:
  CarController();

  ~CarController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();

  using Params = car_controller::Params; // defined in car_controller_parameters.hpp
  using ParamListener = car_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  CmdType commands_in_;
  void callback_command(const CmdType::SharedPtr msg);
  rclcpp::Subscription<CmdType>::SharedPtr subscriber_command_;

  std::vector<double> commands_out_;
  void set_actuator_commands(const std::vector<double> & commands);
  
  std::vector<double> get_states();
};
}  // namespace simple_car_controller

#endif  // DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_