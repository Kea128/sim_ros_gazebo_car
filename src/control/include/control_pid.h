#ifndef _CONTROL_PID_H
#define _CONTROL_PID_H

namespace control_algorithm {
namespace pid {
class PID {
 public:
  // 构造函数
  PID(double Kp, double Ki, double Kd, double maxOutput, double minOutput,
      double integralLimit = 0)
      : Kp_(Kp),
        Ki_(Ki),
        Kd_(Kd),
        maxOutput_(maxOutput),
        minOutput_(minOutput),
        integralLimit_(integralLimit),
        integral_(0.0),
        prev_error_(0.0) {}
  // ~PID();

  void setPID(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  }

  // 更新PID控制器
  double update(const double &setValue, const double &feedBack);

 private:
  double Kp_;             // 比例增益
  double Ki_;             // 积分增益
  double Kd_;             // 微分增益
  double maxOutput_;      // 最大输出
  double minOutput_;      // 最小输出
  double integralLimit_;  // 积分限幅(+)
  double integral_;       // 积分项
  double prev_error_;     // 上一次的误差
};
}  // namespace pid
}  // namespace control_algorithm

#endif