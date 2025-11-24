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

   ImuMessageCallBack callback_;
   //std::array<uint8_t,SERIAL_MSG_SIZE> buffer_;
   // 替换原来的 fixed buffer
   std::vector<uint8_t> recv_buffer_;  // 动态接收缓冲区
   boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false> crc16_;


   void do_read();
   void try_sync_and_parse();
   bool parse_message(const uint8_t*data);
};


}
#endif  // SERIAL_RECEIVER_HPP