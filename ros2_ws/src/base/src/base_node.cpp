#include "base/base_node.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "base/protocol/interact_cmds.hpp"

namespace base {

namespace {

static_assert(sizeof(float) == 4, "KFS 串口协议要求 float 占 4 字节");

// 将一个 float 拆成协议规定的小端四字节。
void append_float32_le(std::vector<uint8_t> & bytes, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int shift = 0; shift < 32; shift += 8) {
    bytes.push_back(static_cast<uint8_t>((bits >> shift) & 0xFFU));
  }
}

}  // namespace

BaseNode::BaseNode(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<std::string>("port_name", "ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<std::string>("kfs_target_topic", "kfs/target");
  get_parameter("port_name", port_name_);
  get_parameter("baudrate", baudrate_);
  get_parameter("kfs_target_topic", kfs_target_topic_);

  const std::string device = "/dev/" + port_name_;
  RCLCPP_INFO(get_logger(), "port: %s, baudrate: %d", device.c_str(), baudrate_);

  std::string error;
  if (serial_.open(device, static_cast<uint32_t>(baudrate_), error)) {
    RCLCPP_INFO(get_logger(), "Serial port opened");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", error.c_str());
    throw std::runtime_error("Failed to open serial port " + device + ": " + error);
  }

  kfs_target_subscription_ = create_subscription<custom_msgs::msg::KfsTarget>(
    kfs_target_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&BaseNode::send_kfs_target, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Base node started (KFS target topic=%s; no heartbeat or ACK)",
    kfs_target_subscription_->get_topic_name());
}

void BaseNode::send_kfs_target(const custom_msgs::msg::KfsTarget::SharedPtr message)
{
  if (message->color != custom_msgs::msg::KfsTarget::BLUE &&
    message->color != custom_msgs::msg::KfsTarget::RED)
  {
    RCLCPP_WARN(
      get_logger(), "Discarding KFS target with invalid color=%u",
      static_cast<unsigned int>(message->color));
    return;
  }
  if (!std::isfinite(message->x_m) || !std::isfinite(message->y_m) ||
    !std::isfinite(message->yaw_rad))
  {
    RCLCPP_WARN(get_logger(), "Discarding KFS target with non-finite pose");
    return;
  }

  // DATA：第 0 字节是颜色，随后依次写入 x、y、yaw 的小端四字节 float。
  std::vector<uint8_t> data;
  data.reserve(13);
  data.push_back(message->color);
  append_float32_le(data, message->x_m);
  append_float32_le(data, message->y_m);
  append_float32_le(data, message->yaw_rad);

  const protocol::UartFrame frame{protocol::uart_cmd::KFS_TARGET, std::move(data)};
  std::string error;
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (!serial_.write(frame.encode(), error)) {
    RCLCPP_ERROR(get_logger(), "KFS target write failed: %s", error.c_str());
  }
}

}  // namespace base
