#ifndef BASE_HPP_
#define BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>

#include <string>
#include <cmath>
#include <chrono>
#include <cstring>   // memcpy

class Base : public rclcpp::Node
{
public:
  explicit Base(const std::string & node_name = "base");  // 构造函数，禁用隐式转换
  ~Base();  // 析构函数

private:
  // 回调
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);  // 传入消息为共享指针
  void timer_callback();

  // 串口发送
  void send_velocity(float vx, float vy, float omega);
  void send_stop();
  

  // 串口对象
  serial::Serial serial_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 参数
  std::string port_name_;
  int baudrate_;
  bool auto_stop_on_;

  // 超时停车计数（100ms 一次，>5 即约 0.5s）
  int auto_stop_count_;
  static constexpr int kAutoStopThreshold = 5;

  // 速度限幅（可按实际修改）
  static constexpr float kMaxVx = 0.5f;      // m/s
  static constexpr float kMaxVy = 0.5f;      // m/s
  static constexpr float kMaxOmega = 0.5f;   // rad/s
};   // 闭合 class Base

#endif  // BASE_HPP_
