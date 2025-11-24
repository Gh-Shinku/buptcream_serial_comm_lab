#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"   
#include "serial_comm/serial_receiver.hpp"
#include <thread>


// TODO: 创建一个 publisher, 调用你实现的 receiver，
// 将受到的 imu 数据以 sensor_msgs/msg/imu 格式发布到一个 topic 中

class ImupublisherNode : public rclcpp::Node {
public:
    ImuPublisherNode() : Node("imu_publisher") {
   
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    std::string serial_port = this->get_parameter("serial_port").as_string();

    receiver_thread_ = std::thread([serial_port, this]() {
      auto receiver = std::make_unique<serial_comm::SerialReceiver>(serial_port,
        [this](const serial_comm::ImuMessage& msg) {
          auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
          
          imu_msg->header.stamp = this->now();
          imu_msg->header.frame_id = "imu_link";

          imu_msg->orientation.w = msg.quaternion.w;
          imu_msg->orientation.x = msg.quaternion.x;
          imu_msg->orientation.y = msg.quaternion.y;
          imu_msg->orientation.z = msg.quaternion.z;

          imu_msg->angular_velocity.x = msg.angular_velocity.x;
          imu_msg->angular_velocity.y = msg.angular_velocity.y;
          imu_msg->angular_velocity.z = msg.angular_velocity.z;

          imu_msg->linear_acceleration.x = msg.linear_acceleration.x;
          imu_msg->linear_acceleration.y = msg.linear_acceleration.y;
          imu_msg->linear_acceleration.z = msg.linear_acceleration.z;

          publisher_->publish(std::move(imu_msg));
        });
      receiver->start();
    });
  }

  ~ImuPublisherNode() {
    if (receiver_thread_.joinable()) {
      receiver_thread_->detach();
    }
  }

private:
  rclcpp::publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  //std::unique_ptr<std::thread>receiver_thread_; 
  std::thread receiver_thread_;
};



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImupublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}