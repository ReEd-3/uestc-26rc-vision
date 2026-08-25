#ifndef BASE_PROTOCOL_UART_INTERACT_HPP_
#define BASE_PROTOCOL_UART_INTERACT_HPP_

#include <cstddef>
#include <cstdint>

#include "base/protocol/interact_cmds.hpp"

namespace base::protocol {

/*
 * 串口字节流解析器。
 * 每收到一个字节调用 push()；返回 true 时可通过 frame() 取得完整帧。
 * 当前 base_node 不接收串口数据，保留此类供以后增加上行消息时使用。
 */
class UartInteract
{
public:
  // 输入一个字节；组成一帧且校验正确时返回 true。
  bool push(uint8_t byte);
  // 返回最近一次成功解析出的帧。
  const UartFrame & frame() const;
  // 放弃当前帧并回到等待帧头的状态。
  void reset();

private:
  // 解析顺序：帧头 -> 命令字 -> 长度 -> 数据 -> 校验和 -> 帧尾。
  enum class State {kWaitHead, kWaitCmd, kWaitLen, kWaitData, kWaitSum, kWaitTail};

  State state_ = State::kWaitHead;
  UartFrame frame_{};
  uint8_t len_ = 0;
  uint8_t sum_ = 0;
  std::size_t data_idx_ = 0;
};

}  // namespace base::protocol

#endif  // BASE_PROTOCOL_UART_INTERACT_HPP_
