#include "base/protocol/interact_cmds.hpp"

namespace base::protocol {

uint8_t UartFrame::checksum(const std::vector<uint8_t> & data)
{
  uint8_t sum = 0;
  for (const uint8_t byte : data) {
    sum ^= byte;
  }
  return sum;
}

std::vector<uint8_t> UartFrame::encode() const
{
  std::vector<uint8_t> bytes;
  bytes.reserve(data.size() + 5);
  bytes.push_back(uart_cmd::HEAD);
  bytes.push_back(cmd);
  bytes.push_back(static_cast<uint8_t>(data.size()));
  bytes.insert(bytes.end(), data.begin(), data.end());
  bytes.push_back(checksum(data));
  bytes.push_back(uart_cmd::TAIL);
  return bytes;
}

}  // namespace base::protocol
