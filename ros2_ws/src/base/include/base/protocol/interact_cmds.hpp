#ifndef BASE_PROTOCOL_INTERACT_CMDS_HPP_
#define BASE_PROTOCOL_INTERACT_CMDS_HPP_

#include <cstdint>
#include <vector>

/**
 * @file interact_cmds.hpp
 * @brief base_node 与下位机之间 UART 帧格式及当前已确认命令的说明。
 *
 * 当前协议只用于确认上位机与下位机的串口链路是否在线：上位机发送
 * HEARTBEAT，下位机回复 ACK。速度控制、任务控制、状态回传和完成事件
 * 均不属于当前协议；未来需要新的下行功能时，确认具体含义后再新增命令字。
 *
 * 帧格式（每一项都是一个字节）：
 * @code
 * 0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB
 * @endcode
 *
 * - HEAD 固定为 0x55，TAIL 固定为 0xBB。
 * - CMD 表示消息用途，例如 0xA0 表示 HEARTBEAT，0x85 表示 ACK。
 * - LEN 表示 DATA 中有多少字节；空 DATA 时 LEN 和 SUM 都为 0。
 * - SUM 是 DATA 中全部字节的逐字节 XOR；HEAD、CMD、LEN、TAIL 均不参与校验。
 * - 接收端按 HEAD、CMD、LEN、DATA、SUM、TAIL 的顺序逐字节解析。校验或
 *   帧尾错误时丢弃当前帧，并重新等待下一处 HEAD。
 *
 * 当前已确认的下行消息（上位机 -> 下位机）：
 * - 0xA0 HEARTBEAT：DATA 为空，因此 LEN=0、SUM=0。完整帧为
 *   0x55 0xA0 0x00 0x00 0xBB；base_node 每 500 ms 发送一次。
 *
 * 当前已确认的上行消息（下位机 -> 上位机）：
 * - 0x85 ACK：DATA 为空，因此 LEN=0、SUM=0。完整帧为
 *   0x55 0x85 0x00 0x00 0xBB，表示下位机已收到最近一次 HEARTBEAT。
 *
 * 修改本说明、帧格式、命令字或 DATA 布局时，必须同步修改下位机固件；本文件
 * 的说明应与 UartFrame::encode()、UartInteract::push() 和电控的实现一致。
 */
namespace base::protocol::uart_cmd {

constexpr uint8_t HEAD = 0x55;
constexpr uint8_t TAIL = 0xBB;

constexpr uint8_t HEARTBEAT = 0xA0;
constexpr uint8_t ACK = 0x85;

}  // namespace base::protocol::uart_cmd

namespace base::protocol {

/** 一帧已完成的 UART 消息，不含 HEAD、LEN、SUM 和 TAIL。 */
struct UartFrame
{
  uint8_t cmd;               ///< 命令字，见 uart_cmd 命名空间。
  std::vector<uint8_t> data; ///< DATA 区原始字节，小端排列；空 DATA 时为空 vector。

  /** 按 HEAD|CMD|LEN|DATA|SUM|TAIL 顺序编码成可直接写串口的字节序列。 */
  std::vector<uint8_t> encode() const;
  /** 对传入的 DATA 区逐字节异或；只用于 DATA，不含 HEAD/CMD/LEN/TAIL。 */
  static uint8_t checksum(const std::vector<uint8_t> & data);
};

}  // namespace base::protocol

#endif  // BASE_PROTOCOL_INTERACT_CMDS_HPP_
