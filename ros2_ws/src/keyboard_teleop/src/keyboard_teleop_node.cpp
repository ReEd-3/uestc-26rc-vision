#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop(const std::string & node_name)
  : Node(node_name)
  {
    // 参数声明，完全对齐 base 节点的参数风格
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<double>("linear_speed", 0.15);
    this->declare_parameter<double>("angular_speed", 0.40);
    this->declare_parameter<double>("command_timeout", 0.35);
    this->declare_parameter<double>("max_linear_velocity", 1.0);
    this->declare_parameter<double>("min_linear_velocity", -1.0);
    this->declare_parameter<double>("max_angular_velocity", 0.5);
    this->declare_parameter<double>("min_angular_velocity", -0.5);

    this->get_parameter("cmd_vel_topic", topic_name_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("angular_speed", angular_speed_);
    this->get_parameter("command_timeout", command_timeout_);
    this->get_parameter("max_linear_velocity", max_linear_);
    this->get_parameter("min_linear_velocity", min_linear_);
    this->get_parameter("max_angular_velocity", max_angular_);
    this->get_parameter("min_angular_velocity", min_angular_);

    if (publish_rate_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "publish_rate must be greater than zero");
      rclcpp::shutdown();
      return;
    }

    // 打开终端，设置 cbreak 模式，符合 Linux 终端交互程序惯例
    stdin_fd_ = fileno(stdin);
    if (!isatty(stdin_fd_)) {
      RCLCPP_ERROR(this->get_logger(), "Keyboard teleop requires an interactive terminal");
      rclcpp::shutdown();
      return;
    }

    // 保存原始终端设置
    tcgetattr(stdin_fd_, &original_settings_);
    new_settings_ = original_settings_;
    new_settings_.c_lflag &= ~(ICANON | ECHO);
    new_settings_.c_lflag |= ISIG;
    new_settings_.c_cc[VMIN] = 0;
    new_settings_.c_cc[VTIME] = 0;
    tcsetattr(stdin_fd_, TCSANOW, &new_settings_);

    // 创建发布器
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);

    // 定时器：20Hz 持续发布当前速度
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&KeyboardTeleop::publish_callback, this));

    // 定时器：50Hz 轮询键盘输入
    keyboard_timer_ = this->create_wall_timer(
      20ms,
      std::bind(&KeyboardTeleop::keyboard_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Keyboard teleop started -> topic=%s, publish_rate=%.1f Hz, "
      "linear_speed=%.2f m/s, angular_speed=%.2f rad/s",
      topic_name_.c_str(), publish_rate_, linear_speed_, angular_speed_);
    print_help();
  }

  ~KeyboardTeleop() override
  {
    send_stop();
    // 恢复原始终端设置，避免退出后 shell 输入卡死
    tcsetattr(stdin_fd_, TCSADRAIN, &original_settings_);
    RCLCPP_INFO(this->get_logger(), "Keyboard teleop exited");
  }

private:
  void print_help()
  {
    puts("\n"
      "===== Base 底盘键盘遥控（C++，完全对齐现有 ROS2 base 节点）=====\n"
      "  W : 前进       vx 正方向\n"
      "  S : 后退       vx 负方向\n"
      "  A : 左转       omega 正方向\n"
      "  D : 右转       omega 负方向\n"
      "  空格 : 立即全部停止\n"
      "  Q   : 全部停止并退出程序\n"
      "\n"
      "速度限幅：\n"
      "  vx 范围 [-1.0, 1.0] m/s\n"
      "  omega 范围 [-0.5, 0.5] rad/s\n"
      "  按键超时：松开后最多 0.35s 自动停车\n"
      "\n");
    fflush(stdout);
  }

  int read_key()
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(stdin_fd_, &fds);

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int ret = select(stdin_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      return -1;
    }

    unsigned char c = 0;
    if (read(stdin_fd_, &c, 1) <= 0) {
      return -1;
    }
    return tolower(c);
  }

  void keyboard_callback()
  {
    int key = read_key();
    if (key < 0) {
      return;
    }

    bool handled = true;
    switch(key) {
      case 'w':
        vx_ = linear_speed_;
        vy_ = 0.0;
        omega_ = 0.0;
        break;
      case 's':
        vx_ = -linear_speed_;
        vy_ = 0.0;
        omega_ = 0.0;
        break;
      case 'a':
        vx_ = 0.0;
        vy_ = 0.0;
        omega_ = angular_speed_;
        break;
      case 'd':
        vx_ = 0.0;
        vy_ = 0.0;
        omega_ = -angular_speed_;
        break;
      case ' ':
        vx_ = 0.0;
        vy_ = 0.0;
        omega_ = 0.0;
        break;
      case 'q':
        send_stop();
        rclcpp::shutdown();
        return;
      default:
        handled = false;
        break;
    }

    if (!handled) {
      return;
    }

    // 限幅，和 base.cpp 中 cmd_vel_callback 的写法完全保持一致
    vx_ = std::max(std::min(vx_, max_linear_), min_linear_);
    vy_ = std::max(std::min(vy_, max_linear_), min_linear_);
    omega_ = std::max(std::min(omega_, max_angular_), min_angular_);

    last_command_time_ = std::chrono::steady_clock::now();
    timeout_stopped_ = false;
  }

  void publish_callback()
  {
    // 按键超时自动停车，松开后不按键，0.35s 就停，防止小车失控乱跑
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      now - last_command_time_).count();

    if (command_timeout_ > 0.0 && elapsed > command_timeout_) {
      if ((std::fabs(vx_) > 1e-4 || std::fabs(vy_) > 1e-4 || std::fabs(omega_) > 1e-4)
          && !timeout_stopped_) {
        vx_ = 0.0;
        vy_ = 0.0;
        omega_ = 0.0;
        timeout_stopped_ = true;
        RCLCPP_WARN(this->get_logger(), "Keyboard command timed out, auto stop");
      }
    }

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = vx_;
    msg.linear.y = vy_;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = omega_;
    cmd_vel_pub_->publish(msg);
  }

  void send_stop()
  {
    // 连续发 5 帧零速度，降低网络丢包概率，和 base 节点析构停车风格一致
    for (int i = 0; i < 5; i++) {
      auto msg = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(msg);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;

  std::string topic_name_;
  double publish_rate_;
  double linear_speed_;
  double angular_speed_;
  double command_timeout_;
  double max_linear_;
  double min_linear_;
  double max_angular_;
  double min_angular_;

  double vx_{0.0};
  double vy_{0.0};
  double omega_{0.0};
  bool timeout_stopped_{false};
  std::chrono::steady_clock::time_point last_command_time_;

  int stdin_fd_;
  struct termios original_settings_;
  struct termios new_settings_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>("keyboard_teleop");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
