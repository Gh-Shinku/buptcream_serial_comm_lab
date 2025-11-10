#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <array>
#include <functional>
#include <string>
#include <cstring>
#include <spdlog/spdlog.h>
#include "serial_comm/message.hpp"

namespace serial_comm {
class SerialReceiver {
  // TODO
  using SerialBuffer = std::array<uint8_t,serial_comm::SERIAL_MSG_SIZE>;
  using CallBack = std::function<void(const ImuMessage& )>;
private:
  boost::asio::io_context ioc_;
  boost::asio::serial_port serial_;

  SerialBuffer buffer_;

  boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false> crc_16;

  CallBack callback_;

  void async_read();
  void parse_buffer();

public:
  explicit SerialReceiver(const std::string& port);

  void setCallback(CallBack callback);

  void start();
};
}

#endif  // SERIAL_RECEIVER_HPP