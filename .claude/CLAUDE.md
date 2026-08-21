# uestc-26rc-vision

- 这是机器人工程比赛项目的**上位机（迷你主机）代码仓库**，基于 **ROS 2 Jazzy**，工作空间为 `ros2_ws/`
- 单片机（下位机 STM32H723ZGT6，CAN 控制 M3508/C620、FreeRTOS 任务、机械臂/吸盘控制）的固件代码**不在本仓库**，在另一个仓库中维护。本仓库负责与下位机之间的 USART 链路，以及（规划中的）视觉识别和手动遥控
- 改动 `base` 包里的协议实现前，先读下文「USART 协议」一节，并与下位机仓库的实现核对一致（尤其是校验和算法，两边必须完全一致）
- `base` 包目前正在重构：只保留了心跳/ACK 链路检测这一项已确认功能，速度控制、事件上报、状态回传等旧功能已被移除，等待重新设计后逐步加回（见下文）；`keyboard_teleop` 包已被删除，尚无替代

## 比赛任务背景（供理解上下文，非本仓库代码范围）

机器人一共要完成三个任务：

1. 从启动区移动到指定位置，该位置有一个三个方块（每个边长35cm，650g左右）竖向堆叠的"塔"，机器人要在不推倒塔的情况下把最上方的方块上下翻转180°
2. 完成任务1后要经过一个高低不一的四平台（高的距离地面40cm，低的距离地面20cm），有三个相同颜色（颜色已知）和一个不同颜色的方块，识别出需要取的三个颜色相同的方块，经过一个15°、水平1.5m的向上斜坡把方块运送到斜坡上的地面
3. 把运送后的方块堆叠成一个竖向放置的塔

底盘为麦克纳姆轮*4 + DJI M3508*4/C620，机械臂为可伸缩升降臂*2（M3508驱动伸缩升降 + GO8010关节旋转），末端吸盘（一臂1个/一臂2个，电磁阀控制），视觉/寻路用2D激光雷达 + 双目摄像头。这些均由下位机及尚未加入本仓库的视觉节点承担；本仓库当前只有底盘串口链路节点。

## 仓库结构（`ros2_ws/src/`）

| 包 | 说明 |
|----|------|
| `base` | 底盘串口链路节点 `base_node`：与下位机做 UART 心跳/ACK 链路检测。核心逻辑在本仓库 |
| `serial` | 第三方 [wjwwood/serial](https://github.com/wjwwood/serial) 串口库，vendored 进来编译成 ROS 2 静态库，供 `base` 依赖；一般不需要修改 |

视觉（双目摄像头/激光雷达识别、导航决策）目前**还没有对应的包**，尽管仓库名叫 `-vision`——这是待补的部分。`keyboard_teleop`（键盘遥控 `/cmd_vel`）包已被删除，如需手动遥控需要重新设计。

## `base` 包内部分层

```text
base/
├── include/base/
│   ├── base_node.hpp              # ROS 2 节点业务：心跳、ACK、链路状态
│   ├── protocol/                  # 与具体串口和 ROS 无关的协议层
│   │   ├── interact_cmds.hpp      # 帧格式、命令字、DATA 定义（协议唯一入口）
│   │   └── uart_interact.hpp      # 把连续字节解析为 UartFrame
│   └── transport/
│       └── serial_transport.hpp   # 纯字节读写；隔离当前 serial 库
└── src/                            # 与上面的 include 一一对应的实现
```

- `transport::SerialTransport` 只做字节收发，不认识帧格式；换串口库只需要改这一层
- `protocol::UartFrame`/`UartInteract` 只认识通用帧格式（HEAD/CMD/LEN/DATA/SUM/TAIL），不认识 HEARTBEAT、ACK 等业务含义
- `BaseNode`（`base` 命名空间）是业务层，持有一个 `SerialTransport` + 一个 `UartInteract`，用独立的 `rx_thread_` 线程阻塞读串口并喂给解析器

## USART 通信协议（`base` 包内实现，见 `include/base/protocol/interact_cmds.hpp` + `uart_interact.hpp`/`.cpp`）

- **帧格式**：`0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB`
  - LEN = DATA 字节数
  - **SUM = 只对 DATA 区逐字节异或**（不含 HEAD/CMD/LEN，也不含 TAIL）——`UartFrame::checksum()` 只接收 `data` 这一个 vector；解析状态机 `UartInteract::push()` 里 `sum_` 也是从进入 `kWaitData` 时清零、每收一个 DATA 字节才异或一次。**这一点务必与下位机固件的校验和实现保持一致**，否则两端会互相判校验失败
  - 数据小端排列

- **命令字**（`uart_cmd` 命名空间，`interact_cmds.hpp`）——当前只确认了链路检测这一对，其余命令字（速度控制、事件上报、状态回传等）已从协议中移除，等重新确认语义后再加：

  | 方向 | CMD | 数据域 | 含义 | 本仓库现状 |
  |------|-----|--------|------|------|
  | 下行 | 0xA0 `HEARTBEAT` | 空 | 上位机心跳 | **已实现**：`BaseNode::heartbeat_timer_callback()`，500ms 周期发送，完整帧 `0x55 0xA0 0x00 0x00 0xBB` |
  | 上行 | 0x85 `ACK` | 空 | 下位机确认收到心跳 | **已实现**：`BaseNode::handle_rx_frame()` 只接受空 DATA 的 ACK，完整帧 `0x55 0x85 0x00 0x00 0xBB` |

- **链路监控（`BaseNode` 内）**：`heartbeat_timer_callback()` 每 500ms 发一次心跳；若超过 2000ms（`kLinkTimeoutMs`）没收到 ACK，判定断联，置 `link_ok_=false` 并打错误日志；收到 ACK 后恢复 `link_ok_=true` 并打日志 "Link recovered"。`last_ack_time_`/`link_ok_` 由定时器线程和 rx 线程共用，靠 `link_mutex_` 保护

## ROS 2 接口

- `base_node` 当前**不发布也不订阅任何话题**，只在内部维护链路在线状态并写日志；之前的 `/cmd_vel` 订阅、`base/event`/`base/status`/`base/ack` 发布都已随速度控制/事件/状态回传功能一起被移除

## 参数（`base_node`，可通过 `--ros-args -p` 覆盖）

- `port_name`（默认 `"ttyUSB0"`，实际打开 `/dev/<port_name>`）
- `baudrate`（默认 `115200`）
- `heartbeat_period_ms`（默认 `500`，心跳发送周期）
- `link_timeout_ms`（默认 `2000`，超过这么久没收到 ACK 判定断链）

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
```

## 尚待补充 / 已知 TODO

- 视觉识别节点（双目深度识别方块颜色/位置、激光雷达定位校准、上坡导航决策）尚未加入本仓库
- 速度控制（原 0x10 `SET_VELOCITY` + `/cmd_vel` 订阅）、事件上报（原 0x81/0x82/0x83）、状态回传（原 0x84 `STATUS`）都已从协议和 `base_node` 中移除，需要重新确认下位机侧语义后再设计加回
- 停车距离/巡线/塔距离/转向信号/任务控制（原 0x01~0x05）同样未在当前协议中，需要时重新确认再加
- 机械臂（伸缩升降、GO8010关节）、吸盘电磁阀相关的上下行命令和 ROS 2 接口尚未设计
- `keyboard_teleop`（键盘遥控 `/cmd_vel`）包已删除，若要恢复手动遥控需要重新实现，且要接回 `base_node` 新的速度控制接口（目前还不存在）
- `build/`、`install/`、`log/` 已在 `.gitignore` 中忽略，不要提交
