#include "base/protocol/uart_interact.hpp"

namespace base::protocol {

bool UartInteract::push(uint8_t byte)
{
  switch (state_) {
    case State::kWaitHead:
      if (byte == uart_cmd::HEAD) {
        state_ = State::kWaitCmd;
      }
      break;

    case State::kWaitCmd:
      frame_.cmd = byte;
      state_ = State::kWaitLen;
      break;

    case State::kWaitLen:
      len_ = byte;
      frame_.data.clear();
      frame_.data.reserve(len_);
      sum_ = 0;
      data_idx_ = 0;
      // 数据长度为 0 时，直接读取校验和。
      state_ = (len_ == 0) ? State::kWaitSum : State::kWaitData;
      break;

    case State::kWaitData:
      frame_.data.push_back(byte);
      sum_ ^= byte;
      if (++data_idx_ >= len_) {
        state_ = State::kWaitSum;
      }
      break;

    case State::kWaitSum:
      // 校验和只由 DATA 的所有字节 XOR 得到。
      if (byte == sum_) {
        state_ = State::kWaitTail;
      } else {
        reset();
      }
      break;

    case State::kWaitTail:
      if (byte == uart_cmd::TAIL) {
        state_ = State::kWaitHead;
        return true;
      }
      reset();
      break;
  }
  return false;
}

const UartFrame & UartInteract::frame() const
{
  return frame_;
}

void UartInteract::reset()
{
  state_ = State::kWaitHead;
  frame_ = {};
  len_ = 0;
  sum_ = 0;
  data_idx_ = 0;
}

}  // namespace base::protocol
