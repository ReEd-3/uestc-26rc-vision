#ifndef BASE_BASE_NODE_HPP_
#define BASE_BASE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

#include <custom_msgs/msg/kfs_target.hpp>

#include "base/transport/serial_transport.hpp"

namespace base {

/*
 * ROS 2 串口发送节点。
 * 每收到一条 KfsTarget 消息，就打包为 KFS_TARGET 帧并写入串口。
 * 不发送心跳，不接收 ACK，也不判断下位机是否在线。
 */
class BaseNode : public rclcpp::Node
{
public:
  explicit BaseNode(const std::string & node_name = "base");

private:
  // 将一条有效的 KFS 消息打包成 13 字节数据，并以 KFS_TARGET 命令发送。
  void send_kfs_target(const custom_msgs::msg::KfsTarget::SharedPtr message);

  transport::SerialTransport serial_;
  rclcpp::Subscription<custom_msgs::msg::KfsTarget>::SharedPtr kfs_target_subscription_;

  // 即使将来使用多线程执行器，也保证每次写入的是一整帧，不会混入其他帧。
  std::mutex tx_mutex_;

  // ROS 参数，可通过 --ros-args -p 修改。
  std::string port_name_;
  int baudrate_ = 115200;
  std::string kfs_target_topic_;
};

}  // namespace base

#endif  // BASE_BASE_NODE_HPP_
