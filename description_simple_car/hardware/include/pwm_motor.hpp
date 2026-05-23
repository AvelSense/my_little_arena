#ifndef SIMPLE_PWM_MOTOR_HPP
#define SIMPLE_PWM_MOTOR_HPP

#include <string>
#include <cmath>
  
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class PWM_Motor
{
    public:

    float operating_voltage = 0;
    float internal_resistance = 0;
    float motor_constant = 0;
    float gear_speed_division = 0;
    float gear_speed_division_efficiency = 0;
    float stall_current = 0;
    float no_load_current = 0;
    float no_load_speed = 0;
    float internal_friction = 0;
    float modulated_voltage = 0;

    PWM_Motor() = default;

    PWM_Motor(float operating_voltage, 
      float stall_current, 
      float no_load_current, 
      float no_load_speed,
      float gear_speed_division,
      float gear_speed_division_efficiency)
    {
      setup(operating_voltage, 
      stall_current, 
      no_load_current, 
      no_load_speed,
      gear_speed_division,
      gear_speed_division_efficiency) ;
    }

    
    void setup(float operating_voltage, 
      float stall_current, 
      float no_load_current, 
      float no_load_speed,
      float gear_speed_division,
      float gear_speed_division_efficiency) 
    {
      this->operating_voltage = operating_voltage;
      this->stall_current = stall_current;
      this->no_load_current = no_load_current;
      this->no_load_speed = no_load_speed;
      this->gear_speed_division = gear_speed_division;
      this->gear_speed_division_efficiency = gear_speed_division_efficiency;
      this->internal_resistance = operating_voltage / stall_current;
      this->motor_constant = (operating_voltage - internal_resistance * no_load_current) / no_load_speed;
      this->internal_friction = no_load_current * motor_constant / no_load_speed;
      this->modulated_voltage = 0;
    }

    void deactivate(){
      modulated_voltage = 0;
    }

    void write_neutral_pwm(){
      modulated_voltage = 0;
    }

    void apply_torque(float torque_command, float velocity){
      // For simplicity, we assume that the motor can always provide the desired torque, i.e. we ignore the current limit of the motor.
      float friction = internal_friction * sgn(velocity); 
      float motor_target_torque = torque_command / gear_speed_division / gear_speed_division_efficiency;
      modulated_voltage = (motor_target_torque + friction) / motor_constant * internal_resistance 
      + motor_constant * velocity;
    }

};


#endif // SIMPLE_PWM_MOTOR_HPP