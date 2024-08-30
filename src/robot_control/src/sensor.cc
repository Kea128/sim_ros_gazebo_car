#include <gflags/gflags.h>
#include <glog/logging.h>

#include <array>

#include "control_pid.h"
#include "custom_msgs/control_param.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "quaternion.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

DEFINE_string(log_path, "./log", "Log file path");

float wheelVelL = 0;
float wheelVelR = 0;
float setAngulaVel = 0;
float setLinearVel = 0;

ros::ServiceClient client;
ros::Publisher wheelLeftCmd;
ros::Publisher wheelRightCmd;
ros::Publisher odomPub;

void pub_odom(const sensor_msgs::Imu& imu);
using publishOdomType = std::function<void(const sensor_msgs::Imu&)>;
publishOdomType publishOdom = std::bind(&pub_odom, std::placeholders::_1);

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
  publishOdom(*imu);
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

void pub_odom(const sensor_msgs::Imu& imu) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.seq = 0;
  odom.header.stamp = ros::Time::now();

  odom.pose.pose.orientation.x = imu.orientation.x;
  odom.pose.pose.orientation.y = imu.orientation.y;
  odom.pose.pose.orientation.z = imu.orientation.z;
  odom.pose.pose.orientation.w = imu.orientation.w;
  odomPub.publish(odom);
}

int main(int argc, char* argv[]) {
  // 解析命令行参数
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // 初始化日志库
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "sensor");
  ros::NodeHandle nh;

  // LOG(ERROR) << "Hello, World!";
  // 创建控制客户端<
  client = nh.serviceClient<custom_msgs::control_param>("PID_control");
  ros::service::waitForService("PID_control");

  // 创建关节力矩发布对象
  wheelLeftCmd = nh.advertise<std_msgs::Float64>(
      "/swim_bot/wheel_left_effort_controller/command", 1);
  wheelRightCmd = nh.advertise<std_msgs::Float64>(
      "/swim_bot/wheel_right_effort_controller/command", 1);
  odomPub = nh.advertise<nav_msgs::Odometry>("/hwa/odom", 1);

  // 创建IMU订阅对象
  ros::Subscriber imuSub =
      nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, callback_imu);

  //创建关节速度订阅对象
  ros::Subscriber jointVelSub = nh.subscribe<sensor_msgs::JointState>(
      "/swim_bot/joint_states", 1, callback_jointstate);

  //创建目标速度订阅对象
  ros::Subscriber targetVelSub =
      nh.subscribe<geometry_msgs::Twist>("/my_cmd_vel", 1, callback_keyboard);

  ros::spin();
  return 0;
}
