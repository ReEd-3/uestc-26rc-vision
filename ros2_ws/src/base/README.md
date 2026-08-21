# base

`base_node` 是上位机与下位机之间的 UART 链路节点。当前只实现链路检测：
周期性发送 `HEARTBEAT`，收到对应的 `ACK` 后认为链路在线。

## 目录职责

```text
base/
├── include/base/
│   ├── base_node.hpp              # ROS 2 节点业务：心跳、ACK、链路状态
│   ├── protocol/                  # 与具体串口和 ROS 无关的协议层
│   │   ├── interact_cmds.hpp      # 帧格式、命令字、DATA 定义（协议唯一入口）
│   │   └── uart_interact.hpp      # 把连续字节解析为 UartFrame
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

当前已确认的消息只有：

```text
上位机 -> 下位机  HEARTBEAT  CMD=0xA0, DATA=[]
下位机 -> 上位机  ACK        CMD=0x85, DATA=[]
```

帧格式：

```text
0x55 | CMD | LEN | DATA | SUM | 0xBB
```

因此当前两种完整帧分别是：

```text
HEARTBEAT: 0x55 0xA0 0x00 0x00 0xBB
ACK:       0x55 0x85 0x00 0x00 0xBB
```

## 参数（`--ros-args -p` 覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port_name` | `ttyUSB0` | 实际打开的设备是 `/dev/<port_name>` |
| `baudrate` | `115200` | 串口波特率 |
| `heartbeat_period_ms` | `500` | HEARTBEAT 发送周期 |
| `link_timeout_ms` | `2000` | 超过这么久没收到 ACK 判定断链 |

```bash
ros2 run base base_node --ros-args \
  -p port_name:=ttyUSB0 \
  -p baudrate:=115200 \
  -p heartbeat_period_ms:=500 \
  -p link_timeout_ms:=2000
```

## 链路状态与日志

- 启动时打印端口/波特率、串口是否打开成功、RX 线程是否启动
- 心跳发送失败、串口读失败会各自打一条 `RCLCPP_ERROR`
- 超过 `link_timeout_ms` 未收到 ACK：打印一次 `Link lost`（同一次断联期间不会重复刷屏，哪怕从启动起就从未收到过 ACK 也会报一次，而不是永远沉默）；之后只要收到 ACK 就会打印 `Link recovered` 并自动恢复，不需要重启节点
- **注意区分两种断联**：
  - **逻辑断联**（对端不回 ACK，但串口本身仍然可读写）——上面这套自愈机制能处理，收到 ACK 就自动恢复
  - **物理断联**（串口设备被拔出、`serial_.read()` 报错）——`rx_thread_loop()` 的读线程会直接退出且不会自动重启/重连；此后即使把设备插回去，也不会再收到任何 ACK，只能重启 `base_node` 进程
- 目前 `link_ok_`/链路状态**不对外发布任何 ROS 话题**，纯粹靠日志观测，也没有任何断链后的保护动作（比如停车），因为速度控制功能本身还未接入这个节点

## 新增下行业务命令时怎么改

1. 先与电控确认 CMD、DATA 和 ACK 规则；
2. 在 `protocol/interact_cmds.hpp` 写入命令常量和说明；
3. 在 `BaseNode` 中添加对应的 ROS 接口与发送方法；
4. 保持 `UartInteract` 不变，除非基础帧格式本身改变。

## 更换串口库

不要修改协议或 `BaseNode`。只替换
`src/transport/serial_transport.cpp` 中 `SerialTransport` 的实现即可。
