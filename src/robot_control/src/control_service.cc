#include "control_pid.h"
#include "custom_msgs/control_param.h"
#include "ros/ros.h"

#define outputLimit 1

bool pushTorque(custom_msgs::control_param::Request &req,
                custom_msgs::control_param::Response &res) {
  static control_algorithm::pid::PID velPID(0.01, 0.01, 0, outputLimit,
                                            -outputLimit, 30);
  static control_algorithm::pid::PID turnPID(0.01, 0.01, 0, outputLimit,
                                             -outputLimit, 30);
  double pwmBaseL = velPID.update(req.setLinearVel, req.wheelVelL);
  double pwmBaseR = velPID.update(req.setLinearVel, req.wheelVelR);
  double pwmturn = turnPID.update(req.setAngulaVel, req.imuYawVel);

  // double pwmL = pwmBase - pwmturn;
  // double pwmR = pwmBase + pwmturn;
  double pwmL = pwmBaseL;
  double pwmR = pwmBaseR;

  res.leftTorque = pwmL;
  res.rightTorque = pwmR;
  ROS_INFO("wheelVelL: %f, wheelVelR: %f", req.wheelVelL, req.wheelVelR);
  ROS_INFO("leftTorque: %f , rightTorque: %f", pwmL, pwmR);
  return true;
}

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "control_service_PID");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("PID_control", pushTorque);
  ROS_INFO("*****控制服务已启动*****");
  ros::spin();
  return 0;
}
