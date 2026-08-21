#ifndef BASE_PROTOCOL_UART_INTERACT_HPP_
#define BASE_PROTOCOL_UART_INTERACT_HPP_

#include <cstddef>
#include <cstdint>

#include "base/protocol/interact_cmds.hpp"

namespace base::protocol {

/**
 * @brief UART 字节流解析器。
 *
 * 每收到一个字节调用 push()；返回 true 时，frame() 可取得刚解析完成的帧。
 * 此类只认识通用帧格式，不认识 HEARTBEAT、ACK 等业务含义。
 */
class UartInteract
{
public:
  /** 喂入一个字节；解出完整且校验通过的一帧时返回 true，此时 frame() 有效。 */
  bool push(uint8_t byte);
  /** 最近一次 push() 返回 true 时解析出的帧；其余时候内容未定义。 */
  const UartFrame & frame() const;
  /** 丢弃当前正在解析的帧，回到等待 HEAD 的状态；校验失败或帧尾错误时自动调用。 */
  void reset();

private:
  // 状态机严格按 HEAD -> CMD -> LEN -> DATA*LEN -> SUM -> TAIL 顺序推进；
  // 任何一步不符合预期都 reset() 回 kWaitHead，重新等待下一个 HEAD。
  enum class State {kWaitHead, kWaitCmd, kWaitLen, kWaitData, kWaitSum, kWaitTail};

  State state_ = State::kWaitHead;
  UartFrame frame_{};
  uint8_t len_ = 0;
  uint8_t sum_ = 0;
  std::size_t data_idx_ = 0;
};

}  // namespace base::protocol

#endif  // BASE_PROTOCOL_UART_INTERACT_HPP_
