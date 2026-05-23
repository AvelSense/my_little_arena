#ifndef SIMPLE_PWM_SERVO_HPP
#define SIMPLE_PWM_SERVO_HPP

#include <string>
#include <cmath>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class PWM_Servo
{
    public:
    float angle_min = 0;
    float angle_max = 0;
    int pwm_min = 0;
    int pwm_max = 0;
    int pwm_neutral = 0;
    float target_angle = 0;
    int current_pwm = 0;

    PWM_Servo() = default;

    PWM_Servo(float angle_min, float angle_max, float pwm_min, float pwm_max){
       setup(angle_min, angle_max, pwm_min, pwm_max);
       write_neutral_pwm();
    }
    
    void setup(float angle_min, float angle_max, float pwm_min, float pwm_max) 
    {
      this->angle_min = angle_min;
      this->angle_max = angle_max;
      this->pwm_min = pwm_min;
      this->pwm_max = pwm_max;
      this->pwm_neutral = (pwm_max + pwm_min) / 2;
    }

    void deactivate(){
       write_neutral_pwm();
    }

    void write_neutral_pwm(){
      current_pwm = pwm_neutral;
    }

    void set_angle(float angle){
      float angle_clamped = std::max(std::min(angle, angle_max), angle_min);
      float pwm_per_radian = (pwm_max - pwm_min) / (angle_max - angle_min);
      current_pwm = pwm_neutral + angle_clamped * pwm_per_radian;
    }
    
    double read_rotation(){
      int pwm_offset = current_pwm - pwm_neutral;
      double radians_per_pwm = (angle_max - angle_min) / (pwm_max - pwm_min);
      return pwm_offset * radians_per_pwm;
    }
};


#endif // SIMPLE_PWM_MOTOR_HPP