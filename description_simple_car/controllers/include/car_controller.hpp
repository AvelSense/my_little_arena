#ifndef DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_
#define DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_

#include "simple_car_controller_parameters.hpp"

#include <string>
#include <vector>
#include <optional>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace simple_car_controller
{
enum num_hardware_interfaces
{
    STEERING_LEFT,
    STEERING_RIGHT,
    PROPULSION_LEFT,
    PROPULSION_RIGHT,
};
enum num_ref_interfaces
{
    VELOCITY,
    TURNING_RADIUS,
};

class CarController : public controller_interface::ChainableControllerInterface
{
using CmdType = geometry_msgs::msg::TwistStamped;
public:
  CarController();

  ~CarController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Chainable controller replaces update() with the following two functions
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  struct WheelPropulsion
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_position;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_effort;
  };

  struct WheelSteering
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_position;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_position;
  };

  WheelSteering steering_left_;
  WheelSteering steering_right_;
  WheelPropulsion propulsion_left_;
  WheelPropulsion propulsion_right_;

  using Params = car_controller::Params; // defined in car_controller_parameters.hpp
  using ParamListener = car_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Timeout to consider commands old
  rclcpp::Duration cmd_vel_timeout_ = rclcpp::Duration::from_seconds(0.5);
  rclcpp::Time previous_update_timestamp_{0};

  // realtime container for the received velocity command
  realtime_tools::RealtimeThreadSafeBox<std::optional<CmdType>> rt_box_commands_in_;
  // save the last reference in case of unable to get value from box
  std::optional<CmdType> commands_in_; // TODO; custom message type for control input
  rclcpp::Subscription<CmdType>::SharedPtr subscriber_command_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<TwistStamped>> publisher_control_output_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistStamped>>
    rt_publisher_control_output_ = nullptr;
  geometry_msgs::msg::TwistStamped control_output_message_; // TODO; custom message type for control output

   std::vector<std::string> command_interface_names_;  
   std::vector<std::string> state_interface_names_;
};
}  // namespace simple_car_controller

#endif  // DESCRIPTION_SIMPLE_CAR__CAR_CONTROLLER_HPP_