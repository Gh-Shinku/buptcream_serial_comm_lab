#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <boost/asio.hpp>
#include <boost/crc.hpp> 
#include "serial_comm/message.hpp"

namespace serial_comm {

using ImuMessageCallback = std::function<void(const ImuMessage&)>;

class SerialReceiver {
  public:
   SerialReceiver(const std::string&port_name,ImuMessageCallback callback);

   SerialReceiver(const SerialReceiver &) = delete;
   SerialReceiver &operator=(const SerialReceiver &) = delete;

   void start();

  private:
   boost::asio::io_context ioc_;
   boost::asio::serial_port serial_;

   ImuMessageCallback callback_;
   std::array<uint8_t,SERIAL_MSG_SIZE> buffer_;
   boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false> crc16_;


   void do_read();
   bool parse_message(const uint8_t*data);
};


}
#endif  // SERIAL_RECEIVER_HPP