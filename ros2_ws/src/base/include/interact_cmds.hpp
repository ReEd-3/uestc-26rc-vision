#ifndef INTERACT_CMDS_HPP_
#define INTERACT_CMDS_HPP_

#include <stdint.h>
#include <vector>

/**
 * @file interact_cmds.hpp
 * @brief base_node 与下位机之间 UART 协议的唯一命令定义与帧约定。
 *
 * 帧格式（所有字段均为字节）：
 * @code
 * 0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB
 * @endcode
 *
 * - HEAD 固定为 0x55，TAIL 固定为 0xBB。
 * - LEN 是 DATA 的字节数；空数据帧的 LEN 为 0，SUM 为 0。
 * - SUM 是 DATA 全部字节的 XOR，从 0 开始累积；HEAD、CMD、LEN、TAIL
 *   均不参与校验。
 * - 多字节数值按小端排列。当前的 SET_VELOCITY 和 STATUS 使用 IEEE 754
 *   float32，字段顺序分别为 vx/vy/omega 与 x/y/yaw。
 * - 协议没有转义机制。接收端按 HEAD、CMD、LEN、DATA、SUM、TAIL 的顺序
 *   逐字节解析，任一校验或帧尾错误都会丢弃当前帧并重新等待 HEAD。
 *
 * 下行（上位机 -> 下位机）：
 * - 0x10 SET_VELOCITY：DATA 为 3 个 float32，共 12 字节；base_node 已实现。
 * - 0xA0 HEARTBEAT：DATA 为 1 个 uint8 序列号；base_node 每 500 ms 发送。
 * - 0x01 至 0x05 为预留命令，当前仅保留命令字，base_node 不发送。
 *
 * 上行（下位机 -> 上位机）：
 * - 0x81/0x82/0x83：无 DATA 的完成事件。
 * - 0x84 STATUS：DATA 为 x、y、yaw 三个 float32，共 12 字节。
 * - 0x85 ACK：DATA 为被确认的 CMD 和 result 两个 uint8；仅 HEARTBEAT 的
 *   ACK 用于刷新 base_node 的链路存活时间。
 *
 * 修改帧格式、校验规则、命令字或数据布局时，必须同步核对下位机固件；本文件
 * 的说明应与 UartFrame::encode() 和 UartInteract::push() 保持一致。
 */
namespace uart_cmd {
// 帧边界字节。
constexpr uint8_t HEAD = 0x55;
constexpr uint8_t TAIL = 0xBB;

// 下行命令（上位机 -> 下位机）。0x01 至 0x05 当前只定义、不发送。
constexpr uint8_t SET_STOP_DIST = 0x01;
constexpr uint8_t LINE_FOLLOW   = 0x02;
constexpr uint8_t TOWER_DIST    = 0x03;
constexpr uint8_t TURN_SIGNAL   = 0x04;
constexpr uint8_t TASK_CTRL     = 0x05;
constexpr uint8_t SET_VELOCITY  = 0x10;  // f32 vx, f32 vy, f32 omega（小端）
constexpr uint8_t HEARTBEAT     = 0xA0;  // u8 seq

// 上行命令（下位机 -> 上位机）。
constexpr uint8_t TURN1_DONE = 0x81;
constexpr uint8_t TURN2_DONE = 0x82;
constexpr uint8_t TASK_DONE  = 0x83;
constexpr uint8_t STATUS     = 0x84;  // f32 x, f32 y, f32 yaw（小端）
constexpr uint8_t ACK        = 0x85;  // u8 acknowledged_cmd, u8 result
}

/** 一帧已完成的 UART 消息，不含 HEAD、LEN、SUM 和 TAIL。 */
struct UartFrame
{
    uint8_t cmd;
    std::vector<uint8_t> data;
    std::vector<uint8_t> encode() const;
    /** 返回 data 的逐字节 XOR；这就是帧内 SUM 字段。 */
    static uint8_t checksum(const std::vector<uint8_t>& data);
};

#endif
