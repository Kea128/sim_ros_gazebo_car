#include <stdio.h>
#include <termios.h>  // 包含终端I/O控制头文件

#include <iostream>  // 包含标准输入输出头文件

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

static float linear_vel = 0.25;     // 线速度增量
static float angular_vel = 0.1;     // 角速度增量
static float max_linear_vel = 15;   // 最大轮速
static float min_linear_vel = -15;  // 最小轮速
static float max_angular_vel = 2.5;
static float min_angular_vel = -2.5;

// 函数用于从控制台获取一个字符，不阻塞
int GetCh() {
  static struct termios oldt, newt;  // 保存和修改终端属性的结构体
  tcgetattr(STDIN_FILENO, &oldt);    // 获取当前终端属性
  newt = oldt;                       // 复制属性
  newt.c_lflag &= ~(ICANON);         // 修改属性，取消标准输入缓冲
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 立即应用修改
  int c = getchar();                        // 读取一个字符
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复终端属性
  return c;                                 // 返回读取的字符
}

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "keyboard_cmd");

  std::cout << "Keyboard: \n" << std::endl;
  std::cout << "w --- forward velocity: \n" << std::endl;
  std::cout << "s --- backward velocity: \n" << std::endl;
  std::cout << "a --- rotateLeft velocity: \n" << std::endl;
  std::cout << "d --- rotateRight velocity: \n" << std::endl;
  std::cout << "space --- stop: \n" << std::endl;
  std::cout << "x --- exit: \n" << std::endl;

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub =
      n.advertise<geometry_msgs::Twist>("/my_cmd_vel", 10);

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.linear.z = 0;
  base_cmd.angular.x = 0;
  base_cmd.angular.y = 0;
  base_cmd.angular.z = 0;

  while (n.ok()) {
    int cKey = GetCh();  // 从控制台读取一个字符
    if (cKey == 'w') {
      base_cmd.linear.x += linear_vel;
      if (base_cmd.linear.x < 0) base_cmd.linear.x = 0;
      if (base_cmd.linear.x > max_linear_vel)
        base_cmd.linear.x = max_linear_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    } else if (cKey == 's') {
      base_cmd.linear.x += -linear_vel;
      if (base_cmd.linear.x > 0) base_cmd.linear.x = 0;
      if (base_cmd.linear.x < min_linear_vel)
        base_cmd.linear.x = min_linear_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    } else if (cKey == 'a') {
      base_cmd.angular.z += angular_vel;
      if (base_cmd.angular.z < 0) base_cmd.angular.z = 0;
      if (base_cmd.angular.z > max_angular_vel)
        base_cmd.angular.z = max_angular_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    } else if (cKey == 'd') {
      base_cmd.angular.z += -angular_vel;
      if (base_cmd.angular.z > 0) base_cmd.angular.z = 0;
      if (base_cmd.angular.z < min_angular_vel)
        base_cmd.angular.z = min_angular_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    } else if (cKey == ' ') {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
    } else if (cKey == 'x') {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",
             base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
      printf("Exit! \n");
      return 0;
    } else {
      printf(" - Undefined Instruction \n");
    }
  }
  return 0;
}