#ifndef UART_INTERACT_HPP_
#define UART_INTERACT_HPP_

#include "interact_cmds.hpp"

class UartInteract 
{
public:
    // 逐字节喂入，返回是否凑齐一帧
    bool push(uint8_t byte);
    // 取完整帧（cmd + data）
    const UartFrame& frame() const;
    void reset();
private:
    // 帧解析枚举
    enum class State { kWaitHead, kWaitCmd, kWaitLen, kWaitData, kWaitSum, kWaitTail };
    State state_ = State::kWaitHead;
    UartFrame frame_;
    uint8_t len_ = 0, sum_ = 0;
    size_t data_idx_ = 0;
};

#endif 