#include "base/base_node.hpp"

#include "base/protocol/interact_cmds.hpp"

namespace base {

BaseNode::BaseNode(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<std::string>("port_name", "ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<int>("heartbeat_period_ms", 500);
  declare_parameter<int>("link_timeout_ms", 2000);
  get_parameter("port_name", port_name_);
  get_parameter("baudrate", baudrate_);
  get_parameter("heartbeat_period_ms", heartbeat_period_ms_);
  get_parameter("link_timeout_ms", link_timeout_ms_);

  const std::string device = "/dev/" + port_name_;
  RCLCPP_INFO(get_logger(), "port: %s, baudrate: %d", device.c_str(), baudrate_);

  std::string error;
  if (serial_.open(device, static_cast<uint32_t>(baudrate_), error)) {
    RCLCPP_INFO(get_logger(), "Serial port opened");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", error.c_str());
  }

  last_ack_time_ = std::chrono::steady_clock::now();
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(heartbeat_period_ms_),
    std::bind(&BaseNode::heartbeat_timer_callback, this));

  if (serial_.is_open()) {
    rx_running_ = true;
    rx_thread_ = std::thread(&BaseNode::rx_thread_loop, this);
    RCLCPP_INFO(get_logger(), "RX thread started");
  }
  RCLCPP_INFO(get_logger(), "Base node started (heartbeat and ACK only)");
}

BaseNode::~BaseNode()
{
  rx_running_ = false;
  serial_.close();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void BaseNode::send_heartbeat()
{
  // 空 DATA 的 HEARTBEAT：55 A0 00 00 BB。
  const protocol::UartFrame frame{protocol::uart_cmd::HEARTBEAT, {}};
  std::string error;
  if (!serial_.write(frame.encode(), error)) {
    RCLCPP_ERROR(get_logger(), "Heartbeat write failed: %s", error.c_str());
  }
}

void BaseNode::heartbeat_timer_callback()
{
  send_heartbeat();

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(link_mutex_);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_ack_time_).count();
  // 用 link_lost_logged_ 而不是 link_ok_ 来判断要不要报错：这样无论是
  // “曾经在线后断开”还是“从启动起就没收到过 ACK”，超时后都会报一次，
  // 且同一次断联期间不会每个心跳周期都重复刷屏。
  if (elapsed > link_timeout_ms_ && !link_lost_logged_) {
    link_ok_ = false;
    link_lost_logged_ = true;
    RCLCPP_ERROR(get_logger(), "Link lost (no ACK for %ld ms)", elapsed);
  }
}

void BaseNode::rx_thread_loop()
{
  uint8_t buffer[64];
  while (rx_running_) {
    // serial_.read() 内部有超时（见 SerialTransport::open），不会永久阻塞，
    // 所以析构时设 rx_running_=false 后线程能在一个超时周期内退出并被 join。
    std::string error;
    const std::size_t received = serial_.read(buffer, sizeof(buffer), error);
    if (!error.empty()) {
      if (rx_running_) {
        RCLCPP_ERROR(get_logger(), "Serial read failed: %s", error.c_str());
      }
      break;
    }

    for (std::size_t i = 0; i < received; ++i) {
      if (uart_parser_.push(buffer[i])) {
        handle_rx_frame(uart_parser_.frame());
      }
    }
  }
}

void BaseNode::handle_rx_frame(const protocol::UartFrame & frame)
{
  // 当前唯一接受的上行消息：空 DATA 的 ACK；其余 cmd 或非空 DATA 直接丢弃，
  // 不当作错误处理——协议扩展后这里要按 cmd 分支处理，现在先保持保守。
  if (frame.cmd != protocol::uart_cmd::ACK || !frame.data.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(link_mutex_);
  last_ack_time_ = std::chrono::steady_clock::now();
  if (!link_ok_) {
    link_ok_ = true;
    link_lost_logged_ = false;
    RCLCPP_INFO(get_logger(), "Link recovered");
  }
}

}  // namespace base
