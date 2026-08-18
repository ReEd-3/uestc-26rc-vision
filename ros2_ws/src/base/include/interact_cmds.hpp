#ifndef INTERACT_CMDS_HPP_
#define INTERACT_CMDS_HPP_

#include <stdint.h>
#include <vector>

namespace uart_cmd {
// 帧格式
constexpr uint8_t HEAD = 0x55;
constexpr uint8_t TAIL = 0xBB;

// 下行命令
constexpr uint8_t SET_STOP_DIST = 0x01;
constexpr uint8_t LINE_FOLLOW   = 0x02;
constexpr uint8_t TOWER_DIST    = 0x03;
constexpr uint8_t TURN_SIGNAL   = 0x04;
constexpr uint8_t TASK_CTRL     = 0x05;
constexpr uint8_t SET_VELOCITY  = 0x10;
constexpr uint8_t HEARTBEAT     = 0xA0;

// 上行命令
constexpr uint8_t TURN1_DONE = 0x81;
constexpr uint8_t TURN2_DONE = 0x82;
constexpr uint8_t TASK_DONE  = 0x83;
constexpr uint8_t STATUS     = 0x84;
constexpr uint8_t ACK        = 0x85;
}

struct UartFrame
{
    uint8_t cmd;
    std::vector<uint8_t> data;
    std::vector<uint8_t> encode() const;  // 帧打包
    static uint8_t checksum(const std::vector<uint8_t>& frame);  // 帧校验
};

#endif