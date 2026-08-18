#include "base.hpp"
#include <vector>
#include <stdint.h>
#include "uart_interact.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// 构造函数
Base::Base(const std::string & node_name)
: Node(node_name)
{
  // 参数声明与获取, 串口设备名，波特率，调试日志
  this->declare_parameter<std::string>("port_name", "ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);  // 波特率设置115200
  this->declare_parameter<bool>("debug_log_on", true);  // 手动调试时打印详细日志

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("debug_log_on", debug_log_on_);

  // 日志打印INFO
  RCLCPP_INFO(this->get_logger(),
    "port: /dev/%s, baudrate: %d, debug_log: %s",
    port_name_.c_str(), baudrate_, debug_log_on_ ? "true" : "false");

  // 打开串口
  try {
    serial_.setPort("/dev/" + port_name_);
    serial_.setBaudrate(baudrate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (serial::IOException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
  }

  if (serial_.isOpen()) {
    RCLCPP_INFO(this->get_logger(), "Serial port opened");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
  }

  // 订阅 /cmd_vel（事件触发）
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&Base::cmd_vel_callback, this, _1));

  // 上行发布者
  event_pub_ = this->create_publisher<std_msgs::msg::UInt8>("base/event", 10);
  status_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("base/status", 10);
  ack_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("base/ack", 10);

  // 10ms 轮询接收队列，在 ROS 线程里发布
  rx_timer_ = this->create_wall_timer(
    10ms, std::bind(&Base::rx_timer_callback, this));

  // 心跳定时器：周期发 0xA0 + 断联检测
  last_ack_time_ = std::chrono::steady_clock::now();
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(kHeartbeatPeriodMs),
    std::bind(&Base::heartbeat_timer_callback, this));

  // 启动接收线程（仅串口打开时）
  if (serial_.isOpen()) {
    rx_running_ = true;
    rx_thread_ = std::thread(&Base::rx_thread_loop, this);
    RCLCPP_INFO(this->get_logger(), "RX thread started");
  } else {
    RCLCPP_ERROR(this->get_logger(), "RX thread not started (serial not open)");
  }

  RCLCPP_INFO(this->get_logger(), "Base node started (event-triggered, no odom)");
}

Base::~Base()
{
  send_stop();
  rx_running_ = false;
  if (serial_.isOpen()) {
    serial_.close();   // 关闭 fd，中断阻塞中的 read，让接收线程退出
  }
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void Base::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float vx = static_cast<float>(msg->linear.x);
  float vy = static_cast<float>(msg->linear.y);
  float omega = static_cast<float>(msg->angular.z);

  // 限幅
  vx = std::max(std::min(vx, kMaxVx), -kMaxVx);
  vy = std::max(std::min(vy, kMaxVy), -kMaxVy);
  omega = std::max(std::min(omega, kMaxOmega), -kMaxOmega);

  // 立刻下发
  send_velocity(vx, vy, omega);
}

// 发送函数
void Base::send_velocity(float vx, float vy, float omega)
{
  // 二进制协议：
  std::vector<uint8_t> data;
  data.resize(12);

  // 小端 float32
  std::memcpy(data.data(),  &vx,    4);
  std::memcpy(data.data() + 4,  &vy,    4);
  std::memcpy(data.data() + 8, &omega, 4);

  // 校验：数据区XOR
  UartFrame frame{uart_cmd::SET_VELOCITY, data};
  auto bytes = frame.encode();

  try {
    if (serial_.isOpen()) {
      serial_.write(bytes.data(), bytes.size());
      // 调试日志：手动调试时打印限幅后的速度 + 原始帧字节
      if (debug_log_on_) {
        bool is_zero = (std::fabs(vx) < 1e-4f &&
                        std::fabs(vy) < 1e-4f &&
                        std::fabs(omega) < 1e-4f);
        if (is_zero) {
          // 零速度帧（停车帧）节流：1秒内最多打印一次，防止刷屏
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[TX] STOP (zero velocity)");
        } else {
          // 非零速度帧：每次打印，实时反馈
          std::string hex;
          char buf[8];
          for (size_t i = 0; i < bytes.size(); ++i) {
            std::snprintf(buf, sizeof(buf), "%02X ", bytes[i]);
            hex += buf;
          }
          RCLCPP_INFO(this->get_logger(),
            "[TX] vx=%.3f vy=%.3f omega=%.3f | frame: %s",
            vx, vy, omega, hex.c_str());
        }
      } else {
        // 非调试模式：1秒内最多打印一次，防止刷屏
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Sent cmd: vx=%.3f vy=%.3f omega=%.3f", vx, vy, omega);
      }
    }
  } catch (serial::IOException & e) {
    RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
  }
}

void Base::send_stop()
{
  send_velocity(0.0f, 0.0f, 0.0f);
}

// 发送心跳帧：0x55 | 0xA0 | 0x01 | seq | SUM | 0xBB
void Base::send_heartbeat()
{
  std::vector<uint8_t> data = {heartbeat_seq_};
  UartFrame frame{uart_cmd::HEARTBEAT, data};
  auto bytes = frame.encode();

  try {
    if (serial_.isOpen()) {
      serial_.write(bytes.data(), bytes.size());
    }
  } catch (serial::IOException & e) {
    RCLCPP_ERROR(this->get_logger(), "Heartbeat write failed: %s", e.what());
  }
}

// 心跳定时器：发心跳 + 断联检测
void Base::heartbeat_timer_callback()
{
  // 发送心跳，seq 递增（u8 溢出自动回绕）
  send_heartbeat();
  heartbeat_seq_++;

  // 断联检测：超过 kLinkTimeoutMs 未收到回执
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_ack_time_).count();

  if (elapsed > kLinkTimeoutMs) {
    if (link_ok_) {
      // 首次判定断联：告警 + 停车
      link_ok_ = false;
      RCLCPP_ERROR(this->get_logger(),
        "Link lost (no ACK for %ld ms), sending stop", elapsed);
      send_stop();
    }
  } else {
    link_ok_ = true;
  }
}

// 接收线程：阻塞读串口 → 喂状态机 → 完整帧入队
void Base::rx_thread_loop()
{
  uint8_t buf[64];
  while (rx_running_) {
    size_t n = 0;
    try {
      n = serial_.read(buf, sizeof(buf));
    } catch (serial::IOException & e) {
      RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", e.what());
      break;
    }
    for (size_t i = 0; i < n; ++i) {
      if (uart_parser_.push(buf[i])) {
        UartFrame f = uart_parser_.frame();   // 拷贝，避免引用悬垂
        {
          std::lock_guard<std::mutex> lock(rx_mutex_);
          rx_queue_.push_back(std::move(f));
        }
      }
    }
  }
  RCLCPP_WARN(this->get_logger(), "RX thread exited");
}

// 轮询接收队列，在 ROS 线程里发布
void Base::rx_timer_callback()
{
  std::deque<UartFrame> frames;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    frames.swap(rx_queue_);   // 一次性取空，减少锁竞争
  }
  for (auto & f : frames) {
    handle_rx_frame(f);
  }
}

// 按 cmd 分发上行帧
void Base::handle_rx_frame(const UartFrame & frame)
{
  switch (frame.cmd) {
    case uart_cmd::TURN1_DONE:
    case uart_cmd::TURN2_DONE:
    case uart_cmd::TASK_DONE: {
      auto msg = std_msgs::msg::UInt8();
      msg.data = frame.cmd;
      event_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Event received: 0x%02X", frame.cmd);
      break;
    }
    case uart_cmd::STATUS: {
      if (frame.data.size() >= 12) {
        float x, y, yaw;
        std::memcpy(&x, frame.data.data(), 4);
        std::memcpy(&y, frame.data.data() + 4, 4);
        std::memcpy(&yaw, frame.data.data() + 8, 4);
        auto msg = geometry_msgs::msg::Pose2D();
        msg.x = x;
        msg.y = y;
        msg.theta = yaw;
        status_pub_->publish(msg);
      }
      break;
    }
    case uart_cmd::ACK: {
      if (frame.data.size() >= 2) {
        // 心跳回执：data[0]==0xA0 时更新链路确认时间
        if (frame.data[0] == uart_cmd::HEARTBEAT) {
          last_ack_time_ = std::chrono::steady_clock::now();
          if (!link_ok_) {
            link_ok_ = true;
            RCLCPP_INFO(this->get_logger(), "Link recovered");
          }
        }
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = {frame.data[0], frame.data[1]};
        ack_pub_->publish(msg);
      }
      break;
    }
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown cmd: 0x%02X", frame.cmd);
      break;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Base>("base");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
