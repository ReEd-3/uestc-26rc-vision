#ifndef BASE_PROTOCOL_INTERACT_CMDS_HPP_
#define BASE_PROTOCOL_INTERACT_CMDS_HPP_

#include <cstdint>
#include <vector>

/*
 * base_node 与下位机的串口协议说明。
 *
 * 当前协议只包含上位机下行的 KFS_TARGET 位姿消息；不使用 HEARTBEAT 或 ACK。
 * 速度控制、任务控制、状态回传和完成事件均不属于当前协议；未来需要新功能时，
 * 确认具体含义后再新增命令字。
 *
 * 帧格式（每一项都是一个字节）：
 * 0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB
 *
 * - 帧头固定为 0x55，帧尾固定为 0xBB。
 * - CMD 是命令字；当前 0x20 表示 KFS 目标。
 * - LEN 是 DATA 的字节数。
 * - SUM 是 DATA 所有字节逐个 XOR 的结果；帧头、命令字、长度和帧尾不参与计算。
 *
 * KFS 目标下行消息（上位机 -> 下位机）：
 * - 0x20 KFS_TARGET：DATA 固定为 13 字节，因此完整帧为
 *   0x55 0x20 0x0D DATA[13] SUM 0xBB。base_node 每收到一条有效的
 *   `custom_msgs/msg/KfsTarget` 就发送一帧；持续识别到 KFS 时持续发送，
 *   不做去重、仅首次触发或超时失效处理。
 *
 *   DATA 的字节布局（小端序）：
 *   字节位置  长度  类型      字段       含义
 *      0       1    uint8_t   color      0=蓝色，1=红色
 *      1       4    float32   x_m        相对相机右为正，单位 m
 *      5       4    float32   y_m        相对相机前为正，单位 m
 *      9       4    float32   yaw_rad    KFS 向相机右侧偏为正，单位 rad
 *
 *   例如 `1.0f` 为`00 00 80 3F`。SUM 仍只对上述 13 个 DATA 字节逐字节 XOR。
 *
 * 修改帧格式、命令字或 DATA 布局时，必须同时修改下位机的解析代码。
 */
namespace base::protocol::uart_cmd {

constexpr uint8_t HEAD = 0x55;
constexpr uint8_t TAIL = 0xBB;

// KFS 目标命令。DATA 固定为 13 字节，字段顺序和单位见本文件上方说明。
constexpr uint8_t KFS_TARGET = 0x20;

}  // namespace base::protocol::uart_cmd

namespace base::protocol {

// 一帧已经编码完成的串口消息，不包含帧头、长度、校验和和帧尾。
struct UartFrame
{
  uint8_t cmd;               // 命令字。
  std::vector<uint8_t> data; // DATA 中的原始字节；没有数据时为空。

  // 编码为可直接写入串口的完整字节序列。
  std::vector<uint8_t> encode() const;
  // 计算 DATA 的逐字节 XOR 校验和。
  static uint8_t checksum(const std::vector<uint8_t> & data);
};

}  // namespace base::protocol

#endif  // BASE_PROTOCOL_INTERACT_CMDS_HPP_
