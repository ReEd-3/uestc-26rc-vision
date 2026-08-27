# UESTC 26RC ROS 2 工作空间

本仓库的 ROS 2 工作空间位于 [`ros2_ws/`](ros2_ws/)，当前包含：

- [`base`](ros2_ws/src/base/README.md)：订阅有效 `/kfs/target` 并向下位机发送 UART KFS 帧；
- [`serial`](ros2_ws/src/serial/README.md)：串口传输库；
- [`kfs_vision`](ros2_ws/src/kfs_vision/README.md)：直接使用 Orbbec RGB-D 和 YOLO-seg 的 KFS 位姿节点。

`kfs_vision` 在同帧存在多个红/蓝 KFS 时，先做 YOLO NMS，再优先选择横向最靠近图像中心的实例；置信度只用于横向偏移相同的排序。发布前还会拒绝与同色上一条有效位姿跳变过大的帧。下游 `base` 只转发已经通过这些视觉校验的有效消息。
