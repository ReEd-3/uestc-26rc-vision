# base

`base_node` 是上位机与下位机之间的 UART 发送节点。它订阅 ROS 的
`/kfs/target`，每收到一条有效目标消息就向下位机发送一帧 KFS 目标数据。
本版本不发送 `HEARTBEAT`、不接收 `ACK`、不维护链路在线状态。

`base_node` 不参与 KFS 实例选择、深度门控或位姿校验；这些由上游
[`kfs_vision`](../kfs_vision/README.md) 完成。它只转发收到的每一条有效 ROS
消息，因此视觉端选择中央 KFS 的规则不会在串口层被再次改变。

## 目录职责

```text
base/
├── include/base/
│   ├── base_node.hpp              # ROS 2 节点业务：订阅 KFS 并发送串口帧
│   ├── protocol/                  # 与具体串口和 ROS 无关的协议层
│   │   ├── interact_cmds.hpp      # 帧格式、命令字、DATA 定义（协议唯一入口）
│   └── transport/
│       └── serial_transport.hpp   # 纯字节读写；隔离当前 serial 库
└── src/
    ├── main.cpp                   # 程序入口，仅启动 ROS 节点
    ├── base_node.cpp              # 当前业务逻辑
    ├── protocol/                  # 协议层实现
    └── transport/                 # 串口库适配实现
```

## 当前协议

完整帧格式和命令定义只看：
[`include/base/protocol/interact_cmds.hpp`](include/base/protocol/interact_cmds.hpp)

当前只发送：

```text
上位机 -> 下位机  KFS_TARGET CMD=0x20, DATA=[color, x_m, y_m, yaw_rad]
```

帧格式：

```text
0x55 | CMD | LEN | DATA | SUM | 0xBB
```

`KFS_TARGET` 的 DATA 固定为 13 字节，详细字段定义以
[`include/base/protocol/interact_cmds.hpp`](include/base/protocol/interact_cmds.hpp)
为准：`color` 占 1 字节，`x_m`、`y_m`、`yaw_rad` 分别占 4 字节小端 `float32`。
因此其帧格式为：

```text
0x55 0x20 0x0D DATA[13] XOR(DATA) 0xBB
```

每收到一条有效 `/kfs/target` 消息就发送一帧，不做去重、仅首次触发或超时失效
处理。持续识别到目标时，KFS 帧会随视觉消息持续发送。

## 参数（`--ros-args -p` 覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port_name` | `ttyUSB0` | 实际打开的设备是 `/dev/<port_name>` |
| `baudrate` | `115200` | 串口波特率 |
| `kfs_target_topic` | `kfs/target` | 订阅的 `custom_msgs/msg/KfsTarget` 话题 |

```bash
ros2 run base base_node --ros-args \
  -p port_name:=ttyUSB0 \
  -p baudrate:=115200 \
  -p kfs_target_topic:=kfs/target
```

## 新增下行业务命令时怎么改

1. 先与电控确认 CMD、DATA 布局和下位机处理规则；
2. 在 `protocol/interact_cmds.hpp` 写入命令常量和说明；
3. 在 `BaseNode` 中添加对应的 ROS 接口与发送方法；
4. 保持 `UartInteract` 不变，除非基础帧格式本身改变。

## 更换串口库

不要修改协议或 `BaseNode`。只替换
`src/transport/serial_transport.cpp` 中 `SerialTransport` 的实现即可。
