#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "serial_comm/serial_receiver.hpp"
#include "serial_comm/message.hpp"

// TODO: 创建一个 publisher, 调用你实现的 receiver，将受到的 imu 数据以 sensor_msgs/msg/imu 格式发布到一个 topic 中
using namespace serial_comm;

class YourNode : public rclcpp::Node {
private:
  std::unique_ptr<SerialReceiver> receive_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
public:
  YourNode() : rclcpp::Node("node_name") 
  {
    pub = this -> create_publisher<sensor_msgs::msg::Imu>("/imu/data",10);
    receive_ = std::make_unique<SerialReceiver>("/dev/ttyUSB1");

    receive_ -> setCallback([this](const ImuMessage& imu_data)
    {
      sensor_msgs::msg::Imu msg;

      msg.header.stamp = this -> now();
      msg.header.frame_id = "ium/link";

      msg.orientation.w = imu_data.quaternion.w;
      msg.orientation.x = imu_data.quaternion.x;
      msg.orientation.y = imu_data.quaternion.y;
      msg.orientation.z = imu_data.quaternion.z;

      msg.angular_velocity.x = imu_data.angular_velocity.x;
      msg.angular_velocity.y = imu_data.angular_velocity.y;
      msg.angular_velocity.z = imu_data.angular_velocity.z;

      msg.angular_velocity.x = imu_data.angular_velocity.x;
      msg.angular_velocity.y = imu_data.angular_velocity.y;
      msg.angular_velocity.z = imu_data.angular_velocity.z;

      pub -> publish(msg);
    });

    RCLCPP_INFO(this -> get_logger(),"Imu note start");
    receive_ -> start();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YourNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}