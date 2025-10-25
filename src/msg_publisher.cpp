#include "serial_comm/serial_receiver.hpp"
#include <CLI11/CLI11.hpp>
#include <serial_comm/message.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>  

class ImuPublisher {
public:
    ImuPublisher(const std::string& port, int baud_rate) {
        // Create serial receiver
        try {
            receiver_ = std::make_unique<serial_comm::SerialReceiver>(port, baud_rate);
            
            receiver_->set_message_callback([this](const serial_comm::SerialMessage& msg) {
                this->print_imu_data(msg.data);
            });
            
            receiver_->start();
            std::cout << "Serial receiver started on port: " << port << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize serial receiver: " << e.what() << std::endl;
            throw;
        }
    }

    void run() {
        while (true) {
            // Just keep running to receive data
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void print_imu_data(const serial_comm::ImuMessage& imu_data) {
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);
        
        std::cout << "\n--- IMU Data [" << std::put_time(std::localtime(&now_time), "%T") << "] ---\n";
        
        // Print quaternion data
        std::cout << "Orientation (quaternion):\n";
        std::cout << "  w: " << imu_data.quaternion.w << "\n";
        std::cout << "  x: " << imu_data.quaternion.x << "\n";
        std::cout << "  y: " << imu_data.quaternion.y << "\n";
        std::cout << "  z: " << imu_data.quaternion.z << "\n";
        
        // Print angular velocity
        std::cout << "Angular velocity (rad/s):\n";
        std::cout << "  x: " << imu_data.angular_velocity.x << "\n";
        std::cout << "  y: " << imu_data.angular_velocity.y << "\n";
        std::cout << "  z: " << imu_data.angular_velocity.z << "\n";
        
        // Print linear acceleration
        std::cout << "Linear acceleration (m/s²):\n";
        std::cout << "  x: " << imu_data.linear_acceleration.x << "\n";
        std::cout << "  y: " << imu_data.linear_acceleration.y << "\n";
        std::cout << "  z: " << imu_data.linear_acceleration.z << "\n";
        
        std::cout << std::endl;
    }

    std::unique_ptr<serial_comm::SerialReceiver> receiver_;
};

int main(int argc, char** argv) {
    CLI::App app{"IMU Data Publisher"};
    
    // Add command line options
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;
    
    app.add_option("-p,--port", port, "Serial port device")->check(CLI::ExistingFile);
    app.add_option("-b,--baud", baud_rate, "Baud rate")->check(CLI::PositiveNumber);
    
    CLI11_PARSE(app, argc, argv);
    
    try {
        ImuPublisher publisher(port, baud_rate);
        publisher.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}