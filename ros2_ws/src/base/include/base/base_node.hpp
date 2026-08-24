#ifndef BASE_BASE_NODE_HPP_
#define BASE_BASE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <custom_msgs/msg/kfs_target.hpp>

#include "base/protocol/uart_interact.hpp"
#include "base/transport/serial_transport.hpp"

namespace base {

/**
 * @brief ROS 2 节点业务层。
 *
 * 当前业务包含 HEARTBEAT/ACK 链路检测，以及将 KfsTarget 下行发送给下位机。
 * 协议编解码位于 protocol，底层串口读写位于 transport；本类不直接依赖具体
 * 串口库。
 */
class BaseNode : public rclcpp::Node
{
public:
  explicit BaseNode(const std::string & node_name = "base");
  ~BaseNode() override;

private:
  /** 定时器回调：发送一次心跳，并检查距上次收到 ACK 是否已超时。 */
  void heartbeat_timer_callback();
  /** 编码并发送一帧空 DATA 的 HEARTBEAT。 */
  void send_heartbeat();
  /** 将一条有效 KfsTarget 打包为 13 字节 DATA，并以 KFS_TARGET 命令发送。 */
  void send_kfs_target(const custom_msgs::msg::KfsTarget::SharedPtr message);
  /** 独立线程的主循环：阻塞读串口，逐字节喂给 uart_parser_，解出完整帧就转交业务处理。 */
  void rx_thread_loop();
  /** 处理一帧已解析完成的上行消息；当前只识别空 DATA 的 ACK，其余帧直接忽略。 */
  void handle_rx_frame(const protocol::UartFrame & frame);

  transport::SerialTransport serial_;
  protocol::UartInteract uart_parser_;
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Subscription<custom_msgs::msg::KfsTarget>::SharedPtr kfs_target_subscription_;

  // 心跳定时器与 KFS 订阅回调都可能写串口；同一时刻只允许写入完整的一帧。
  std::mutex tx_mutex_;

  // 定时器和接收线程都会访问下列链路状态，需同一把锁保护。
  std::mutex link_mutex_;
  std::chrono::steady_clock::time_point last_ack_time_;
  bool link_ok_ = false;
  // 断联是否已经报过日志：避免每个心跳周期重复刷屏，也保证从未收到过
  // ACK（link_ok_ 从启动起一直是 false）时也能报一次，而不是永远沉默。
  bool link_lost_logged_ = false;

  // 以下均为 ROS 参数（构造函数里 declare_parameter），可通过 --ros-args -p 覆盖。
  std::string port_name_;
  int baudrate_ = 115200;
  int heartbeat_period_ms_ = 500;
  int link_timeout_ms_ = 2000;
  std::string kfs_target_topic_;
};

}  // namespace base

#endif  // BASE_BASE_NODE_HPP_
