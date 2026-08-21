# uestc-26rc-vision

- 这是机器人工程比赛项目的**上位机（迷你主机）代码仓库**，基于 **ROS 2 Jazzy**，工作空间为 `ros2_ws/`
- 单片机（下位机 STM32H723ZGT6，CAN 控制 M3508/C620、FreeRTOS 任务、机械臂/吸盘控制）的固件代码**不在本仓库**，在另一个仓库中维护。本仓库只负责：通过 USART 协议下发速度指令、接收下位机上行事件与状态，以及（规划中的）视觉识别与键盘手动遥控
- 改动 `base` 包里的协议实现前，先读下文「USART 协议」一节，并与下位机仓库的实现核对一致（尤其是校验和算法，两边必须完全一致）

## 比赛任务背景（供理解上下文，非本仓库代码范围）

机器人一共要完成三个任务：

1. 从启动区移动到指定位置，该位置有一个三个方块（每个边长35cm，650g左右）竖向堆叠的"塔"，机器人要在不推倒塔的情况下把最上方的方块上下翻转180°
2. 完成任务1后要经过一个高低不一的四平台（高的距离地面40cm，低的距离地面20cm），有三个相同颜色（颜色已知）和一个不同颜色的方块，识别出需要取的三个颜色相同的方块，经过一个15°、水平1.5m的向上斜坡把方块运送到斜坡上的地面
3. 把运送后的方块堆叠成一个竖向放置的塔

底盘为麦克纳姆轮*4 + DJI M3508*4/C620，机械臂为可伸缩升降臂*2（M3508驱动伸缩升降 + GO8010关节旋转），末端吸盘（一臂1个/一臂2个，电磁阀控制），视觉/寻路用2D激光雷达 + 双目摄像头。这些均由下位机及尚未加入本仓库的视觉节点承担；本仓库当前只有底盘串口驱动和键盘遥控。

## 仓库结构（`ros2_ws/src/`）

| 包 | 说明 |
|----|------|
| `base` | 底盘驱动节点 `base_node`：订阅 `/cmd_vel`、串口收发下位机协议、发布上行事件/状态/ACK。核心逻辑在本仓库 |
| `keyboard_teleop` | 键盘遥控节点 `keyboard_teleop_node`：读键盘 → 发布 `/cmd_vel`，用于手动调试底盘，不依赖串口，可独立测试 |
| `serial` | 第三方 [wjwwood/serial](https://github.com/wjwwood/serial) 串口库，vendored 进来编译成 ROS 2 静态库，供 `base` 依赖；一般不需要修改 |

视觉（双目摄像头/激光雷达识别、导航决策）目前**还没有对应的包**，尽管仓库名叫 `-vision`——这是待补的部分。

## USART 通信协议（`base` 包内实现，见 `src/base/include/interact_cmds.hpp` + `uart_interact.hpp`/`.cpp` + `base.cpp`）

- **帧格式**：`0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB`
  - LEN = DATA 字节数
  - **SUM = 只对 DATA 区逐字节异或**（不含 HEAD/CMD/LEN，也不含 TAIL）——`UartFrame::checksum()` 只接收 `data` 这一个 vector；解析状态机 `UartInteract::push()` 里 `sum_` 也是从进入 `kWaitData` 时清零、每收一个 DATA 字节才异或一次。**这一点务必与下位机固件的校验和实现保持一致**，否则两端会互相判校验失败
  - 数据小端排列

- **命令字**（`uart_cmd` 命名空间，`interact_cmds.hpp`）：

  | 方向 | CMD | 数据域 | 含义 | 本仓库现状 |
  |------|-----|--------|------|------|
  | 下行 | 0x01 `SET_STOP_DIST` | — | 目标停车距离 | 仅定义常量，`base_node` 尚未发送 |
  | 下行 | 0x02 `LINE_FOLLOW` | — | 巡线数据 | 仅定义常量，尚未发送 |
  | 下行 | 0x03 `TOWER_DIST` | — | 到塔距离 | 仅定义常量，尚未发送 |
  | 下行 | 0x04 `TURN_SIGNAL` | — | T路口/右转信号 | 仅定义常量，尚未发送 |
  | 下行 | 0x05 `TASK_CTRL` | — | 任务控制 | 仅定义常量，尚未发送 |
  | 下行 | 0x10 `SET_VELOCITY` | f32 vx + f32 vy + f32 omega（小端，12字节） | 手动/自动调速度 | **已实现**：`Base::send_velocity()`，由 `/cmd_vel` 回调触发，即时下发，带限幅（kMaxVx=1.0 m/s, kMaxVy=1.0 m/s, kMaxOmega=0.5 rad/s）|
  | 下行 | 0xA0 `HEARTBEAT` | u8 seq | 上位机心跳 | **已实现**：`Base::heartbeat_timer_callback()`，500ms 周期发送，seq 自增回绕 |
  | 上行 | 0x81 `TURN1_DONE` / 0x82 `TURN2_DONE` / 0x83 `TASK_DONE` | 空 | 事件 | **已实现**：统一发布到 `base/event`（`std_msgs/UInt8`，值为 cmd） |
  | 上行 | 0x84 `STATUS` | f32 x + f32 y + f32 yaw | 状态回传 | **已实现**：发布到 `base/status`（`geometry_msgs/Pose2D`） |
  | 上行 | 0x85 `ACK` | u8 cmd + u8 result | 命令应答 | **已实现**：发布到 `base/ack`（`std_msgs/UInt8MultiArray`，`[cmd, result]`）；若 `cmd==HEARTBEAT` 还会刷新链路存活时间 |

- **链路监控（`base` 节点内）**：`heartbeat_timer_callback()` 每 500ms 发一次心跳；若超过 2000ms（`kLinkTimeoutMs`）没收到心跳 ACK（0x85 且 data[0]==0xA0），判定断联，调用 `send_stop()` 并置 `link_ok_=false`；收到心跳 ACK 后恢复 `link_ok_=true` 并打日志 "Link recovered"

## ROS 2 接口

- **订阅** `/cmd_vel`（`geometry_msgs/Twist`，`base_node`）：`linear.x`→vx, `linear.y`→vy, `angular.z`→omega，限幅后立即打包成 0x10 帧下发，不做缓存/插值
- **发布**（`base_node`）：
  - `base/event`（`std_msgs/UInt8`）：上行 0x81/0x82/0x83 事件码
  - `base/status`（`geometry_msgs/Pose2D`）：上行 0x84 的 x/y/yaw
  - `base/ack`（`std_msgs/UInt8MultiArray`）：上行 0x85 的 `[cmd, result]`
- `keyboard_teleop_node` 只发布 `/cmd_vel`（W/S 前后，A/D 左右转，空格停止，Q 停止并退出；松键 0.35s 超时自动停车），不直接接触串口，可脱离硬件单独测试

## 参数（`base_node`，可通过 `--ros-args -p` 覆盖）

- `port_name`（默认 `"ttyUSB0"`，实际打开 `/dev/<port_name>`）
- `baudrate`（默认 `115200`）
- `debug_log_on`（默认 `true`，开启后每帧非零速度都会打印十六进制帧内容）

## 构建与运行

- ROS 2 发行版：Jazzy；**必须用 shell 里的 `colcon build`，不要用 CMake Tools 插件构建**（见提交历史 `a966e9e`）
- `serial` 包编译为 **静态库**（`STATIC`），供 `base` 链接；构建产物含 `compile_commands.json`，语言服务器配置见 `ros2_ws/.vscode/settings.json` 里的 `C_Cpp.default.compileCommands`（指向 `ros2_ws/src/base/compile_commands.json`，注意里面的绝对路径是他人机器上的，换机器需要自行改）
- 串口设备访问需要用户在 `dialout` 组（见提交 `63a077e`）

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 底盘串口节点
ros2 run base base_node --ros-args -p port_name:=ttyUSB0 -p baudrate:=115200

# 键盘遥控（另开终端，交互式）
ros2 run keyboard_teleop keyboard_teleop_node
```

## 尚待补充 / 已知 TODO

- 视觉识别节点（双目深度识别方块颜色/位置、激光雷达定位校准、上坡导航决策）尚未加入本仓库
- 0x01/0x02/0x03/0x04/0x05（停车距离/巡线/塔距离/转向信号/任务控制）的下行发送逻辑还没接入 `base_node`，目前只是协议常量占位
- 机械臂（伸缩升降、GO8010关节）、吸盘电磁阀相关的上下行命令和 ROS 2 接口尚未设计
- `build/`、`install/`、`log/` 已在 `.gitignore` 中忽略，不要提交
