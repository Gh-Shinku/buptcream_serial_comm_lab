#include "serial_comm/serial_receiver.hpp"

#include <spdlog/spdlog.h>
#include <cstring>

namespace serial_comm{
//反向put_field 读取
template<typename T>
inline T get_field(const uint8_t *buf,size_t &offset)
{
    T val;
    std::memcpy(&val,buf+offset,sizeof(T));
    offset +=sizeof(T);
    return val;
}


 bool SerialReceiver::parse_message(const uint8_t*data)
 {
    size_t offset=0;
    uint8_t head=get_field<uint8_t>(data,offset);
    if(head!=static_cast<uint8_t>(SERIAL_MSG_HEAD))
     return false;

    offset+=sizeof(uint32_t);

    ImuMessage msg;
        msg.quaternion.w = get_field<double>(data, offset);
        msg.quaternion.x = get_field<double>(data, offset);
        msg.quaternion.y = get_field<double>(data, offset);
        msg.quaternion.z = get_field<double>(data, offset);
        msg.angular_velocity.x = get_field<double>(data, offset);
        msg.angular_velocity.y = get_field<double>(data, offset);
        msg.angular_velocity.z = get_field<double>(data, offset);
        msg.linear_acceleration.x = get_field<double>(data, offset);
        msg.linear_acceleration.y = get_field<double>(data, offset);
        msg.linear_acceleration.z = get_field<double>(data, offset);

    crc16_.process_bytes(data, CRC_FIELD_SIZE);
    uint16_t computed_crc=crc16_.checksum();
    crc16_.reset();

    uint16_t received_crc=get_field<uint16_t>(data,offset);
    if (computed_crc!=received_crc)
    {
        spdlog::warn("CRC mismatch: computed={:04X}, received={:04X}", computed_crc, received_crc);
        return false;
    }

    uint8_t tail = get_field<uint8_t>(data, offset);
    if (tail != static_cast<uint8_t>(SERIAL_MSG_TAIL))
        return false;
    
    callback_(msg);
    return true;
}

SerialReceiver::SerialReceiver(const std::string&port_name,ImuMessageCallback callback)
: serial_(ioc_, port_name), 
callback_(callback)
{
  serial_.set_option(boost::asio::serial_port::baud_rate(115200));
  serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serial_.set_option(boost::asio::serial_port::character_size(8));
}

void SerialReceiver::do_read() {
    boost::asio::async_read(serial_, boost::asio::buffer(buffer_),
        [this](const boost::system::error_code& ec, std::size_t /*bytes*/) 
        {
            if (!ec) {
                if (parse_message(buffer_.data()))
                {
                    spdlog::debug("Received valid IMU message");
                } 
                else 
                {
                    spdlog::warn("Invalid message received");
                }
                do_read(); // 继续读下一帧
            } 
            else 
            {
                spdlog::error("Read error: {}", ec.message());
            }
        });
}

void SerialReceiver::start()
{
    do_read();
    ioc_.run();
}


}