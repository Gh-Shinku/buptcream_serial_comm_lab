#include "serial_comm/serial_receiver.hpp"
#include <iostream>
#include <stdexcept>

namespace serial_comm {

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

void SerialReceiver::set_message_callback(MessageCallback callback) {
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

void SerialReceiver::handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
        if (error == boost::asio::error::operation_aborted) return;
        std::cerr << "Serial read error: " << error.message() << std::endl;
        return;
    }

    if (bytes_transferred == SERIAL_MSG_SIZE) {
        auto msg = parse_message(receive_buffer_);
        if (validate_message(msg) && message_callback_) {
            message_callback_(msg);
        }
    }

    async_read(); // Continue reading
}

bool SerialReceiver::validate_message(const SerialMessage& msg) const {
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

SerialMessage SerialReceiver::parse_message(const SerialBuffer& buffer) const {
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

} // namespace serial_comm
// TODO