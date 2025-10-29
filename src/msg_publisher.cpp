#include "serial_comm/serial_receiver.hpp"
#include <CLI11/CLI11.hpp>
#include <serial_comm/message.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuPublisher {
public:
    ImuPublisher(const std::string& port, int baud_rate, rclcpp::Node::SharedPtr node)
        : node_(node) {
        // Create serial receiver
        try {
            receiver_ = std::make_unique<serial_comm::SerialReceiver>(port, baud_rate);
            
            // 创建ROS2 IMU消息发布者
            imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
            
            receiver_->set_message_callback([this](const serial_comm::SerialMessage& msg) {
                this->publish_imu_data(msg.data);
            });
            
            receiver_->start();
            RCLCPP_INFO(node_->get_logger(), "Serial receiver started on port: %s", port.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial receiver: %s", e.what());
            throw;
        }
    }

    void run() {
        while (rclcpp::ok()) {
            // 处理ROS2事件，但不会阻塞
            rclcpp::spin_some(node_);
            
            // 保持节点运行
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 停止接收
        if (receiver_) {
            receiver_->stop();
        }
    }

private:
    void publish_imu_data(const serial_comm::ImuMessage& imu_data) {
        // 创建IMU消息
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        
        // 设置时间戳
        msg->header.stamp = node_->now();
        msg->header.frame_id = "imu_link";
        
        // 填充四元数数据
        msg->orientation.w = imu_data.quaternion.w;
        msg->orientation.x = imu_data.quaternion.x;
        msg->orientation.y = imu_data.quaternion.y;
        msg->orientation.z = imu_data.quaternion.z;
        
        // 填充角速度数据
        msg->angular_velocity.x = imu_data.angular_velocity.x;
        msg->angular_velocity.y = imu_data.angular_velocity.y;
        msg->angular_velocity.z = imu_data.angular_velocity.z;
        
        // 填充线加速度数据
        msg->linear_acceleration.x = imu_data.linear_acceleration.x;
        msg->linear_acceleration.y = imu_data.linear_acceleration.y;
        msg->linear_acceleration.z = imu_data.linear_acceleration.z;
        
        // 发布消息
        imu_publisher_->publish(std::move(msg));
    }

    std::unique_ptr<serial_comm::SerialReceiver> receiver_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建ROS2节点
    auto node = rclcpp::Node::make_shared("imu_publisher");
    
    CLI::App app{"IMU Data Publisher"};
    
    // Add command line options
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;
    
    app.add_option("-p,--port", port, "Serial port device")->check(CLI::ExistingFile);
    app.add_option("-b,--baud", baud_rate, "Baud rate")->check(CLI::PositiveNumber);
    
    CLI11_PARSE(app, argc, argv);
    
    try {
        ImuPublisher publisher(port, baud_rate, node);
        
        // 直接运行发布逻辑，不再使用单独的线程和rclcpp::spin
        publisher.run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}