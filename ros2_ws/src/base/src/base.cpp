#include "base.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

Base::Base(const std::string & node_name)
: Node(node_name),
  auto_stop_count_(0)
{
  // 参数声明与获取, 串口设备名，波特率，自动停止的启用
  this->declare_parameter<std::string>("port_name", "ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);  // 波特率设置115200
  this->declare_parameter<bool>("auto_stop_on", true);

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("auto_stop_on", auto_stop_on_);

  // 日志打印INFO
  RCLCPP_INFO(this->get_logger(),
    "port: /dev/%s, baudrate: %d, auto_stop: %s",
    port_name_.c_str(), baudrate_, auto_stop_on_ ? "true" : "false");

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

  // 100ms 定时器，只做超时停车
  timer_ = this->create_wall_timer(
    100ms, std::bind(&Base::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Base node started (event-triggered, no odom)");
}

Base::~Base()
{
  send_stop();
  if (serial_.isOpen()) {
    serial_.close();
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

  // 有有效速度则重置超时计数
  if (std::fabs(vx) > 1e-4f || std::fabs(vy) > 1e-4f || std::fabs(omega) > 1e-4f) {
    auto_stop_count_ = 0;
  }
}

void Base::timer_callback()
{
  if (!auto_stop_on_) {
    return;
  }

  // 未达到超时阈值：计数累加（每 100ms 一次）
  if (auto_stop_count_ < kAutoStopThreshold) {
    auto_stop_count_++;
    return;
  }

  // 达到阈值（约 0.6s 未收到有效速度指令）：保持超时状态，
  // 每 100ms 持续发送停车帧，直到收到新指令（cmd_vel_callback 中清零计数）
  send_stop();
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "cmd_vel timeout, keep sending stop");
}

// 发送函数
void Base::send_velocity(float vx, float vy, float omega)
{
  // 二进制协议：
  // 0x55 | 0x01 | 0x0C | vx(float32 LE) | vy(float32 LE) | omega(float32 LE) | checksum | 0xBB
  uint8_t frame[17];
  frame[0] = 0x55;          // 帧头
  frame[1] = 0x01;          // 命令：速度控制
  frame[2] = 0x0C;          // 数据长度 12 字节

  // 小端 float32
  std::memcpy(&frame[3],  &vx,    4);
  std::memcpy(&frame[7],  &vy,    4);
  std::memcpy(&frame[11], &omega, 4);

  // 校验：数据区累加和
  uint8_t sum = 0;
  for (int i = 3; i < 15; ++i) {
    sum += frame[i];
  }
  frame[15] = sum; 
  frame[16] = 0xBB;         // 帧尾

 try {
    if (serial_.isOpen()) {
      serial_.write(frame, sizeof(frame));
      // 发送成功日志，1秒内最多打印一次，防止刷屏
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Sent cmd: vx=%.3f vy=%.3f omega=%.3f", vx, vy, omega);
    }
  } catch (serial::IOException & e) {
    RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
  }
}

void Base::send_stop()
{
  send_velocity(0.0f, 0.0f, 0.0f);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Base>("base");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
