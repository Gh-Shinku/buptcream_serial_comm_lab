#include "serial_comm/serial_receiver.hpp"
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace serial_comm;


SerialReceiver::SerialReceiver(const std::string& port_name, uint32_t baud_rate) 
    : io_context_(std::make_unique<boost::asio::io_context>()),
      serial_port_(std::make_unique<boost::asio::serial_port>(*io_context_)),
      running_(false) {
    try {
        serial_port_->open(port_name);
        serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial_port_->set_option(boost::asio::serial_port::flow_control(
            boost::asio::serial_port::flow_control::none));
        serial_port_->set_option(boost::asio::serial_port::parity(
            boost::asio::serial_port::parity::none));
        serial_port_->set_option(boost::asio::serial_port::stop_bits(
            boost::asio::serial_port::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port::character_size(8));
    } catch (const boost::system::system_error& e) {
        throw std::runtime_error("Failed to open serial port: " + std::string(e.what()));
    }
}

SerialReceiver::~SerialReceiver() {
    stop();
}

void SerialReceiver::set_message_callback(MessageCallback&& callback) {
    message_callback_ = std::move(callback);
}

void SerialReceiver::start() {
    if (!running_) {
        running_ = true;
        async_read();
        
        // Run IO context in a separate thread
        std::thread([this]() {
            io_context_->run();
        }).detach();
    }
}

void SerialReceiver::stop() {
    if (running_) {
        running_ = false;
        io_context_->stop();
        if (serial_port_->is_open()) {
            serial_port_->close();
        }
    }
}

bool SerialReceiver::is_running() const {
    return running_;
}

void SerialReceiver::async_read() {
    if (!running_) return;

    serial_port_->async_read_some(
        boost::asio::buffer(receive_buffer_),
        [this](const boost::system::error_code& error, size_t bytes_transferred) {
            handle_read(error, bytes_transferred);
        });
}

// 在SerialReceiver类的私有成员中添加以下字段
// (需要在serial_receiver.hpp中也添加对应的声明)
std::vector<uint8_t> message_buffer_; // 用于累积不完整的消息

// 改进的handle_read实现
void SerialReceiver::handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
        if (error == boost::asio::error::operation_aborted) return;
        std::cerr << "Serial read error: " << error.message() << std::endl;
        return;
    }

    // 将新接收的数据添加到消息缓冲区
    message_buffer_.insert(message_buffer_.end(), 
                          receive_buffer_.begin(), 
                          receive_buffer_.begin() + bytes_transferred);

    // 尝试在缓冲区中查找并解析完整的消息
    while (message_buffer_.size() >= SERIAL_MSG_SIZE) {
        // 查找帧头
        auto head_pos = std::find(message_buffer_.begin(), 
                                message_buffer_.begin() + (message_buffer_.size() - SERIAL_MSG_SIZE + 1), 
                                SERIAL_MSG_HEAD);

        if (head_pos == message_buffer_.end()) {
            // 没有找到完整的帧头，保留最后SERIAL_MSG_SIZE-1个字节，其余清空
            if (message_buffer_.size() > SERIAL_MSG_SIZE - 1) {
                std::vector<uint8_t> temp(message_buffer_.end() - (SERIAL_MSG_SIZE - 1), 
                                         message_buffer_.end());
                message_buffer_ = std::move(temp);
            }
            break;
        }

        // 计算帧头位置到缓冲区末尾的距离
        size_t head_idx = head_pos - message_buffer_.begin();
        
        // 如果从帧头开始的剩余数据不足以构成完整消息，退出循环
        if (message_buffer_.size() - head_idx < SERIAL_MSG_SIZE) {
            break;
        }

        // 尝试解析消息
        SerialBuffer temp_buffer;
        std::memcpy(temp_buffer.data(), 
                   message_buffer_.data() + head_idx, 
                   SERIAL_MSG_SIZE);

        auto msg = parse_message(temp_buffer);
        
        // 验证消息的完整性（帧头、帧尾和CRC）
        if (validate_message(msg) && message_callback_) {
            message_callback_(msg);
        }
        
        // 移除已处理的消息（包括帧头前面的垃圾数据）
        message_buffer_.erase(message_buffer_.begin(), head_pos + SERIAL_MSG_SIZE);
    }

    async_read(); // 继续读取
}

bool SerialReceiver::validate_message(const SerialMessage& msg)  {
    if (msg.head != SERIAL_MSG_HEAD || msg.tail != SERIAL_MSG_TAIL) {
        return false;
    }

    // Calculate CRC of the message (excluding CRC field itself)
    crc16_.reset();
    crc16_.process_bytes(&msg.head, sizeof(msg.head));
    crc16_.process_bytes(&msg.id, sizeof(msg.id));
    crc16_.process_bytes(&msg.data, sizeof(msg.data));
    return crc16_.checksum() == msg.crc16;
}

SerialMessage SerialReceiver::parse_message(const SerialBuffer& buffer)  {
    SerialMessage msg;
    size_t offset = 0;
    
    // Manually parse each field to ensure proper alignment
    msg.head = buffer[offset++];
    
    msg.id = *reinterpret_cast<const uint32_t*>(&buffer[offset]);
    offset += sizeof(uint32_t);
    
    msg.data.quaternion.w = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.quaternion.x = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.quaternion.y = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.quaternion.z = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    
    msg.data.angular_velocity.x = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.angular_velocity.y = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.angular_velocity.z = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    
    msg.data.linear_acceleration.x = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.linear_acceleration.y = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    msg.data.linear_acceleration.z = *reinterpret_cast<const double*>(&buffer[offset]);
    offset += sizeof(double);
    
    msg.crc16 = *reinterpret_cast<const uint16_t*>(&buffer[offset]);
    offset += sizeof(uint16_t);
    
    msg.tail = buffer[offset];
    
    return msg;
}

 // namespace serial_comm
// TODO