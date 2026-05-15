#ifndef SIMPLE_WHEEL_HPP
#define SIMPLE_WHEEL_HPP

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"


class Wheel
{
    public:

    int enc = 0;
    double current_pos = 0;
    double current_vel = 0;
    double rads_per_count = 0;

    double current_torque = 0;

    Wheel() = default;

    Wheel(int counts_per_rev)
    {
      setup(counts_per_rev);
    }
    
    void setup(int counts_per_rev)
    {
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    void update_measurement(const rclcpp::Duration & period)
    {
      static std::optional<double> last_pos;
      double current_pos = enc * rads_per_count;

      if (last_pos) { // Need to dereference last_pos since it's an optional
        current_vel = (current_pos - *last_pos) / period.seconds();
      }

      last_pos = current_pos; // But assignment from double to optional<double> is fine, it will just wrap the value in an optional
    }

    double read_rotation()
    {
      return current_pos;
    }

    double read_velocity()
    {
      return current_vel;
    }

    void set_simulated_torque(double torque)
    {
      current_torque = torque;
    }

    void simulate_step(const rclcpp::Duration & period)
    {
      // Simple physics simulation: torque = inertia * angular_acceleration
      // For simplicity, we assume a unit inertia, so angular_acceleration = torque
      double angular_acceleration = current_torque - current_vel; 
      current_vel += angular_acceleration * period.seconds();
      current_pos += current_vel * period.seconds();
    }


};
#endif // SIMPLE_WHEEL_HPP