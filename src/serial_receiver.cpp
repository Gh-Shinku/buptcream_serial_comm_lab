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
    uint32_t id = get_field<uint32_t>(data, offset); // 

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
    recv_buffer_.resize(recv_buffer_.size() + 1);//
    boost::asio::async_read(
        serial_,  boost::asio::buffer(&recv_buffer_.back(), 1),
        [this](const auto& ec, size_t) {
            if (!ec) {
                try_sync_and_parse();  // 尝试帧同步+解析
                do_read();             // 继续读
            } else {
                spdlog::error("Read error: {}", ec.message());
            }
        }
    );
}

void SerialReceiver::try_sync_and_parse()//新增
{
    const size_t FRAME_SIZE = SERIAL_MSG_SIZE; // 88
    while(recv_buffer_.size()>=FRAME_SIZE){
        auto it=std::find(recv_buffer_.begin(),recv_buffer_.end()-FRAME_SIZE+1,
                            static_cast<uint8_t>(SERIAL_MSG_HEAD));

        if(it==recv_buffer_.begin()){//枕头在开头
            if(parse_message(recv_buffer_.data())){
                recv_buffer_.erase(recv_buffer_.begin(),recv_buffer_.begin()+FRAME_SIZE);
                continue;
            }else{// 解析失败
                recv_buffer_.erase(recv_buffer_.begin());
            }
        }else if(it !=recv_buffer_.end()-FRAME_SIZE+1){// 找到帧头但不在开头，丢弃前面的数据
            recv_buffer_.erase(recv_buffer_.begin(),it);
        }else{// 没找到帧头，但缓冲区 >=88
            recv_buffer_.erase(recv_buffer_.begin());
        }
    }
}



void SerialReceiver::start()
{
    do_read();
    ioc_.run();
}


}