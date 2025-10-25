#include "serial_comm/serial_receiver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <chrono>

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher() : Node("imu_publisher") {
        // Parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        
        std::string port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        
        // Create serial receiver
        try {
            receiver_ = std::make_unique<serial_comm::SerialReceiver>(port, baud_rate);
            
            receiver_->set_message_callback([this](const serial_comm::SerialMessage& msg) {
                this->publish_imu(msg.data);
            });
            
            receiver_->start();
            RCLCPP_INFO(this->get_logger(), "Serial receiver started on port: %s", port.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial receiver: %s", e.what());
            rclcpp::shutdown();
        }
    }

private:
    void publish_imu(const serial_comm::ImuMessage& imu_data) {
        auto msg = sensor_msgs::msg::Imu();
        
        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        
        // Set orientation
        msg.orientation.w = imu_data.quaternion.w;
        msg.orientation.x = imu_data.quaternion.x;
        msg.orientation.y = imu_data.quaternion.y;
        msg.orientation.z = imu_data.quaternion.z;
        
        // Set angular velocity
        msg.angular_velocity.x = imu_data.angular_velocity.x;
        msg.angular_velocity.y = imu_data.angular_velocity.y;
        msg.angular_velocity.z = imu_data.angular_velocity.z;
        
        // Set linear acceleration
        msg.linear_acceleration.x = imu_data.linear_acceleration.x;
        msg.linear_acceleration.y = imu_data.linear_acceleration.y;
        msg.linear_acceleration.z = imu_data.linear_acceleration.z;
        
        // Publish
        publisher_->publish(msg);
    }

    std::unique_ptr<serial_comm::SerialReceiver> receiver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisher>());
    rclcpp::shutdown();
    return 0;
}