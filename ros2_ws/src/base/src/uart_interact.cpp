#include "uart_interact.hpp"

//解析接收字节
bool UartInteract::push(uint8_t byte)
{
  switch (state_) {
    case State::kWaitHead:
      if (byte == uart_cmd::HEAD) {
        state_ = State::kWaitCmd;
      }
      // 非 0x55 直接丢弃，继续等帧头
      break;

    case State::kWaitCmd:
      frame_.cmd = byte;
      state_ = State::kWaitLen;
      break;

    case State::kWaitLen:
      len_ = byte;
      frame_.data.clear();
      frame_.data.reserve(len_);
      sum_ = 0;                 // 只对 DATA 区异或，从这开始累计
      data_idx_ = 0;
      state_ = (len_ == 0) ? State::kWaitSum : State::kWaitData;
      break;

    case State::kWaitData:
      frame_.data.push_back(byte);
      sum_ ^= byte;             // 边收边异或
      if (++data_idx_ >= len_) {
        state_ = State::kWaitSum;
      }
      break;

    case State::kWaitSum:
      if (byte == sum_) {       // 校验通过
        state_ = State::kWaitTail;
      } else {
        reset();                // 校验失败，丢弃整帧
      }
      break;

    case State::kWaitTail:
      if (byte == uart_cmd::TAIL) {
        state_ = State::kWaitHead;   // 一帧完整收完
        return true;
      }
      reset();                  // 帧尾不匹配，丢弃
      break;
  }
  return false;
}

// 给出帧的值
const UartFrame& UartInteract::frame() const
{
  return frame_;
}

// 状态重置
void UartInteract::reset()
{
  state_ = State::kWaitHead;
  frame_.cmd = 0;
  frame_.data.clear();
  len_ = 0;
  sum_ = 0;
  data_idx_ = 0;
}