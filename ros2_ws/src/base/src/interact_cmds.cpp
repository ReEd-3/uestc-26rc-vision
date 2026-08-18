#include "interact_cmds.hpp"

uint8_t UartFrame::checksum(const std::vector<uint8_t>& frame) 
{
    uint8_t sum = 0;
    for(auto it = frame.begin(); it != frame.end(); it++) {
        sum ^= *it;
    }
    return sum;
}

std::vector<uint8_t> UartFrame::encode() const 
{
    // 帧格式：0x55 | CMD | LEN | DATA | SUM | 0xBB
    // SUM = 只对 DATA 区逐字节 XOR（与单片机端一致）
    std::vector<uint8_t> frame;
    frame.reserve(data.size() + 5);
    frame.push_back(uart_cmd::HEAD);
    frame.push_back(cmd);
    frame.push_back(static_cast<uint8_t>(data.size()));  // LEN
    frame.insert(frame.end(), data.begin(), data.end()); // DATA
    frame.push_back(checksum(data));                     // SUM（只对 DATA 区）
    frame.push_back(uart_cmd::TAIL);
    return frame;
}