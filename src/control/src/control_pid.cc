#include "control_pid.h"
#include "ros/ros.h"

namespace control_algorithm {
namespace pid {
double PID::update(const double &setValue, const double &feedBack) {
  double error = setValue - feedBack;  // 计算误差
  integral_ += error;                  // 积分项
  // 积分限幅
  integral_ = integral_ > integralLimit_ ? integralLimit_ : integral_;
  integral_ = integral_ < -integralLimit_ ? -integralLimit_ : integral_;

  double derivative = error - prev_error_;  // 微分项

  double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  ROS_INFO("error: %f , integral: %f , output: %f", error, integral_, output);

  output = output > maxOutput_ ? maxOutput_ : output;  // 限制输出
  output = output < minOutput_ ? minOutput_ : output;

  prev_error_ = error;  // 更新上一次的误差

  return output;
}
}  // namespace pid
}  // namespace control_algorithm