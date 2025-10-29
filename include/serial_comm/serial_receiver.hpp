#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <array>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <functional>
#include <memory>
#include "serial_comm/message.hpp"

namespace serial_comm {

class SerialReceiver {
public:
    using SerialBuffer = std::array<uint8_t, SERIAL_MSG_SIZE>;
    using MessageCallback = std::function<void(const SerialMessage&)>;

    SerialReceiver(const std::string& port_name, uint32_t baud_rate = 115200);
    ~SerialReceiver();
    
    SerialReceiver(const SerialReceiver&) = delete;
    SerialReceiver& operator=(const SerialReceiver&) = delete;

    void set_message_callback(MessageCallback&& callback);
    void start();
    void stop();
    bool is_running() const;

private:
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    SerialBuffer receive_buffer_;
    boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false> crc16_;
    MessageCallback message_callback_;
    bool running_;
    std::vector<uint8_t> message_buffer_; // 用于累积不完整的消息

    void async_read();
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
    bool validate_message(const SerialMessage& msg) ;
    SerialMessage parse_message(const SerialBuffer& buffer) ;
};

} // namespace serial_comm

#endif // SERIAL_RECEIVER_HPP