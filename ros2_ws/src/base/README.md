# base

`base_node` 是上位机与下位机之间的 UART 链路节点。当前只实现链路检测：
每 500 ms 发送一次 `HEARTBEAT`，收到对应的 `ACK` 后认为链路在线。

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

## 新增下行业务命令时怎么改

1. 先与电控确认 CMD、DATA 和 ACK 规则；
2. 在 `protocol/interact_cmds.hpp` 写入命令常量和说明；
3. 在 `BaseNode` 中添加对应的 ROS 接口与发送方法；
4. 保持 `UartInteract` 不变，除非基础帧格式本身改变。

## 更换串口库

不要修改协议或 `BaseNode`。只替换
`src/transport/serial_transport.cpp` 中 `SerialTransport` 的实现即可。
