#include <array>

#include "control_pid.h"
#include "custom_msgs/control_param.h"
#include "geometry_msgs/Twist.h"
#include "quaternion.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

float wheelVelL = 0;
float wheelVelR = 0;
float setAngulaVel = 0;
float setLinearVel = 0;

ros::ServiceClient client;
ros::Publisher wheelLeftCmd;
ros::Publisher wheelRightCmd;

void callback_jointstate(const sensor_msgs::JointState::ConstPtr jointState) {
  if (jointState->velocity.size() > 0) {  // 检查消息中是否有速度信息
    wheelVelL = jointState->velocity[0];  //左轮速度获取
    wheelVelR = jointState->velocity[1];  //右轮速度获取
  }
}

void callback_keyboard(const geometry_msgs::Twist::ConstPtr keyboard) {
  setLinearVel = keyboard->linear.x;
  setAngulaVel = keyboard->angular.z;
}

void callback_imu(const sensor_msgs::Imu::ConstPtr imu) {
  static quaternion::Quaternion quat;
  static std::array<float, 3> eulerZYX;
  static std::array<float, 3> eulerZYZ;
  static float roll, pitch, yaw;
  static custom_msgs::control_param srv;

  quat.setQuat(imu->orientation.w, imu->orientation.x, imu->orientation.y,
               imu->orientation.z);
  eulerZYX = quat.toEuler("ZYX");
  eulerZYZ = quat.toEuler("ZYZ");
  roll = eulerZYX[2];
  pitch = eulerZYX[1];
  yaw = eulerZYX[0];

  srv.request.setAngulaVel = setAngulaVel;
  srv.request.setLinearVel = setLinearVel;
  srv.request.wheelVelL = wheelVelL;
  srv.request.wheelVelR = wheelVelR;
  srv.request.imuYawVel = imu->angular_velocity.z;

  bool flag = client.call(srv);
  if (flag) {
    static std_msgs::Float64 leftTorque, rightTorque;
    leftTorque.data = srv.response.leftTorque;
    rightTorque.data = srv.response.rightTorque;
    wheelLeftCmd.publish(leftTorque);
    wheelRightCmd.publish(rightTorque);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sensor");
  ros::NodeHandle nh;

  // 创建控制客户端<
  client = nh.serviceClient<custom_msgs::control_param>("PID_control");
  ros::service::waitForService("PID_control");

  // 创建关节力矩发布对象
  wheelLeftCmd = nh.advertise<std_msgs::Float64>(
      "/swim_bot/wheel_left_effort_controller/command", 1);
  wheelRightCmd = nh.advertise<std_msgs::Float64>(
      "/swim_bot/wheel_right_effort_controller/command", 1);

  // 创建IMU订阅对象
  ros::Subscriber imuSub =
      nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, callback_imu);

  //创建关节速度订阅对象
  ros::Subscriber jointVelSub = nh.subscribe<sensor_msgs::JointState>(
      "/swim_bot/joint_states", 1, callback_jointstate);

  //创建目标速度订阅对象
  ros::Subscriber targetVelSub =
      nh.subscribe<geometry_msgs::Twist>("/my_cmd_vel", 10, callback_keyboard);

  ros::Rate rate(0.05);
  while (ros::ok()) {
    ros::spinOnce();
    ros::Rate rate(0.01);
  }
  return 0;
}
