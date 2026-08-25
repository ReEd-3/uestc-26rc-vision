#ifndef BASE_TRANSPORT_SERIAL_TRANSPORT_HPP_
#define BASE_TRANSPORT_SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace serial {
class Serial;
}

namespace base::transport {

/*
 * 串口读写封装。
 * 这里只负责打开、读取和写入字节，不处理帧头、命令字或校验和。
 * 若更换串口库，只需修改对应的 .cpp 文件。
 */
class SerialTransport
{
public:
  SerialTransport();
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  // 打开串口并设置波特率；失败时返回 false，错误原因写入 error。
  bool open(const std::string & device, uint32_t baudrate, std::string & error);
  bool is_open() const;
  void close();

  // 写入一段字节；端口未打开或发生异常时返回 false。
  bool write(const std::vector<uint8_t> & bytes, std::string & error);
  // 最多读取 capacity 字节，返回实际读取数量。
  std::size_t read(uint8_t * buffer, std::size_t capacity, std::string & error);

private:
  std::unique_ptr<serial::Serial> serial_;
};

}  // namespace base::transport

#endif  // BASE_TRANSPORT_SERIAL_TRANSPORT_HPP_
