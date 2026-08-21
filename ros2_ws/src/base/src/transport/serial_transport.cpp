#include "base/transport/serial_transport.hpp"

#include <serial/serial.h>

namespace base::transport {

SerialTransport::SerialTransport()
: serial_(std::make_unique<serial::Serial>())
{
}

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open(const std::string & device, uint32_t baudrate, std::string & error)
{
  try {
    serial_->setPort(device);
    serial_->setBaudrate(baudrate);
    auto timeout = serial::Timeout::simpleTimeout(2000);
    serial_->setTimeout(timeout);
    serial_->open();
    return true;
  } catch (const serial::IOException & exception) {
    error = exception.what();
    return false;
  }
}

bool SerialTransport::is_open() const
{
  return serial_->isOpen();
}

void SerialTransport::close()
{
  if (serial_->isOpen()) {
    serial_->close();
  }
}

bool SerialTransport::write(const std::vector<uint8_t> & bytes, std::string & error)
{
  try {
    if (!serial_->isOpen()) {
      error = "serial port is not open";
      return false;
    }
    serial_->write(bytes.data(), bytes.size());
    return true;
  } catch (const serial::IOException & exception) {
    error = exception.what();
    return false;
  }
}

std::size_t SerialTransport::read(uint8_t * buffer, std::size_t capacity, std::string & error)
{
  try {
    return serial_->read(buffer, capacity);
  } catch (const serial::IOException & exception) {
    error = exception.what();
    return 0;
  }
}

}  // namespace base::transport
