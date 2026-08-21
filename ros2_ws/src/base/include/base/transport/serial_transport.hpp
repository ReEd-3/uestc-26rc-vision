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

/**
 * @brief 与具体串口库隔离的字节传输层。
 *
 * 不处理帧头、命令字或校验。若以后替换串口库，只需改这个类的 .cpp 文件。
 */
class SerialTransport
{
public:
  SerialTransport();
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  /** 打开设备并设置波特率；失败返回 false 并把异常信息写入 error。 */
  bool open(const std::string & device, uint32_t baudrate, std::string & error);
  bool is_open() const;
  void close();

  /** 整段写入 bytes；失败（含端口未打开）返回 false 并写 error。 */
  bool write(const std::vector<uint8_t> & bytes, std::string & error);
  /** 读取最多 capacity 字节，返回实际读到的字节数；底层带超时，不会永久阻塞。 */
  std::size_t read(uint8_t * buffer, std::size_t capacity, std::string & error);

private:
  std::unique_ptr<serial::Serial> serial_;
};

}  // namespace base::transport

#endif  // BASE_TRANSPORT_SERIAL_TRANSPORT_HPP_
