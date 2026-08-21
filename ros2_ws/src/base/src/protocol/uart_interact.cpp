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
      // LEN=0 时没有 DATA 字节可收，直接跳到 kWaitSum（此时 sum_ 恒为 0）。
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
      // sum_ 是 kWaitData 阶段逐字节异或累积出来的，只覆盖 DATA 区，
      // 必须与下位机固件的校验和算法保持一致（见 interact_cmds.hpp 顶部说明）。
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
