# keyboard_teleop

ROS 2 键盘控制节点，将键盘输入转换为 `geometry_msgs/msg/Twist` 消息并发布到 `/cmd_vel`。

## 功能

- 读取键盘输入；
- 发布底盘速度指令；
- 支持停止操作；
- 退出时发送零速度指令。


## ROS 2 接口

```text
发布话题：/cmd_vel
消息类型：geometry_msgs/msg/Twist
```

使用的字段：

```text
linear.x    前后速度
linear.y    横向速度，通常为 0
angular.z   旋转速度
```

底盘节点负责订阅 `/cmd_vel`，并处理后续的串口和电机控制。

## 编译

```bash
cd ~/uestc-26rc-vision/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select keyboard_teleop --symlink-install
source install/setup.bash
```

如果工作空间路径不同，请替换上面的路径。

## 运行

```bash
source /opt/ros/jazzy/setup.bash
source ~/uestc-26rc-vision/ros2_ws/install/setup.bash
ros2 run keyboard_teleop keyboard_teleop_node
```

具体按键以节点启动时显示的提示为准。

## 验证

在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/uestc-26rc-vision/ros2_ws/install/setup.bash
ros2 topic type /cmd_vel
ros2 topic echo /cmd_vel
```

预期话题类型：

```text
geometry_msgs/msg/Twist
```

查看连接关系：

```bash
ros2 topic info /cmd_vel --verbose
```

只启动键盘节点时，`Subscription count: 0` 是正常的，表示当前没有底盘节点订阅。启动底盘节点后，通常应看到一个发布者和一个订阅者。

## 停止行为

停止操作应发布：

```text
linear.x  = 0.0
linear.y  = 0.0
angular.z = 0.0
```

节点退出时也应尽可能发送零速度。通信超时停车和硬件急停由底盘控制模块负责。

## 模块边界

```text
键盘输入
    ↓
keyboard_teleop_node
    ↓
/cmd_vel
    ↓
底盘控制节点
```

`keyboard_teleop` 只维护键盘输入和 `/cmd_vel` 发布，不依赖串口设备即可独立测试。
