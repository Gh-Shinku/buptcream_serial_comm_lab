#include "serial_comm/serial_receiver.hpp"

// TODO
namespace serial_comm
{

    SerialReceiver::SerialReceiver(const std::string& serialname):serial_(ioc_,serialname)
    {
        serial_.set_option(boost::asio::serial_port::baud_rate(115200));
        serial_.set_option(boost::asio::serial_port::character_size(8));
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    }

    void SerialReceiver::setCallback(CallBack callback)
    {
        callback_ = callback; 
    }

    void SerialReceiver::parse_buffer()
    {
        if (buffer_[0] != SERIAL_MSG_HEAD || buffer_.back() != SERIAL_MSG_TAIL)
        {
            spdlog::error("head or tail error");
            return;
        }
        else
        {
            crc_16.process_bytes(buffer_.data(),CRC_FIELD_SIZE);
            uint16_t crc_calculate = crc_16.checksum();

            uint16_t crc_receive;
            std::memcpy(&crc_receive,buffer_.data() + CRC_FIELD_SIZE,sizeof(uint16_t));

            crc_16.reset();

            if (crc_receive != crc_calculate)
            {
                spdlog::error("crc error, calculate{} , receive{}",crc_calculate,crc_receive);
                return;
            }
            else
            {
                const uint8_t* imu_data = buffer_.data() + 1 + sizeof(uint32_t);
                ImuMessage imu_msg;

                std::memcpy(&imu_msg.quaternion.w,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.quaternion.x,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.quaternion.y,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.quaternion.z,imu_data,sizeof(double));
                imu_data += sizeof(double);
                
                std::memcpy(&imu_msg.angular_velocity.x,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.angular_velocity.y,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.angular_velocity.z,imu_data,sizeof(double));
                imu_data += sizeof(double);
                
                std::memcpy(&imu_msg.linear_acceleration.x,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.linear_acceleration.y,imu_data,sizeof(double));
                imu_data += sizeof(double);
                std::memcpy(&imu_msg.linear_acceleration.z,imu_data,sizeof(double));
                
                if (callback_)
                {
                    callback_(imu_msg);
                }
            }

        }
    }

    void SerialReceiver::async_read()
    {
        boost::asio::async_read(serial_,boost::asio::buffer(buffer_),
    [this](const boost::system::error_code& ec,std::size_t bytes_transferred)
    {
        if (ec)
        {
            spdlog::error("error:{}",ec.message());
            return;
        }
        else
        {
            parse_buffer();
            async_read();
        }  
    });
    }

    void SerialReceiver::start()
    {
        spdlog::info("strat: ");
        async_read();
        ioc_.run();
    }

}