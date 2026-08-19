#ifndef BASE_HPP_
#define BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <serial/serial.h>
#include "uart_interact.hpp"

#include <string>
#include <cmath>
#include <chrono>
#include <cstring>   // memcpy
#include <thread>
#include <mutex>
#include <deque>
#include <atomic>

class Base : public rclcpp::Node
{
public:
  explicit Base(const std::string & node_name = "base");  // 构造函数，禁用隐式转换
  ~Base();  // 析构函数

private:
  // 回调
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);  // 传入消息为共享指针
  void rx_timer_callback();   // 轮询接收队列，在 ROS 线程里发布
  void heartbeat_timer_callback();  // 心跳定时器：发心跳 + 断联检测

  // 串口发送
  void send_velocity(float vx, float vy, float omega);
  void send_stop();
  void send_heartbeat();      // 发送 0xA0 心跳帧（seq 递增）

  // 接收线程
  void rx_thread_loop();                          // 阻塞读串口 → 喂状态机 → 入队
  void handle_rx_frame(const UartFrame & frame);  // 按 cmd 分发（在 ROS 线程调用）

  // 串口对象
  serial::Serial serial_;

  // 接收解析
  UartInteract uart_parser_;
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  std::mutex rx_mutex_;
  std::deque<UartFrame> rx_queue_;   // 接收线程写，ROS 定时器读

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr event_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr ack_pub_;

  // 心跳状态
  uint8_t heartbeat_seq_ = 0;                          // 心跳序列号（递增回绕）
  std::chrono::steady_clock::time_point last_ack_time_;  // 最近一次收到回执的时间
  bool link_ok_ = false;                               // 链路是否正常
  static constexpr int kHeartbeatPeriodMs = 500;       // 心跳周期 500ms
  static constexpr int kLinkTimeoutMs = 2000;          // 2s 未收到回执判定断联

  // 参数
  std::string port_name_;
  int baudrate_;
  bool debug_log_on_;

  // 速度限幅（可按实际修改）
  static constexpr float kMaxVx = 1.0f;      // m/s
  static constexpr float kMaxVy = 1.0f;      // m/s
  static constexpr float kMaxOmega = 0.5f;   // rad/s
};   // 闭合 class Base

#endif  // BASE_HPP_
