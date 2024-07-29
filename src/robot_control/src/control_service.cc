#include "control_pid.h"
#include "custom_msgs/control_param.h"
#include "ros/ros.h"

#define outputLimit 1

control_algorithm::pid::PID velWheelLeftPID(0.01, 0.001, 0, outputLimit,
                                            -outputLimit, 20);
control_algorithm::pid::PID velWheelRightPID(0.01, 0.001, 0, outputLimit,
                                             -outputLimit, 20);
control_algorithm::pid::PID turnPID(0.015, 0.001, 0, outputLimit, -outputLimit,
                                    20);
bool pushTorque(custom_msgs::control_param::Request &req,
                custom_msgs::control_param::Response &res) {
  // ROS_INFO("***************LOOP**************");
  double pwmBaseL = velWheelLeftPID.update(req.setLinearVel, req.wheelVelL);
  double pwmBaseR = velWheelRightPID.update(req.setLinearVel, req.wheelVelR);
  double pwmturn = turnPID.update(req.setAngulaVel, req.imuYawVel);

  double pwmL = pwmBaseL - pwmturn;
  double pwmR = pwmBaseR + pwmturn;
  // double pwmL = pwmBaseL;
  // double pwmR = pwmBaseR;

  res.leftTorque = pwmL;
  res.rightTorque = pwmR;
  // ROS_INFO("wheelVelL: %f, wheelVelR: %f", req.wheelVelL, req.wheelVelR);
  // ROS_INFO("imuYawVel: %f", req.imuYawVel);
  // ROS_INFO("leftTorque: %f , rightTorque: %f", pwmL, pwmR);
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
