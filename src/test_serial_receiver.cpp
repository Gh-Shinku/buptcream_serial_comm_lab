#include "serial_comm/serial_receiver.hpp"
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>

int main() {
  // 创建输出文件来保存处理后的消息
  std::ofstream processed_log("processed_receiver.log");
  
  auto serial_receiver = std::make_unique<serial_comm::SerialReceiver>("/dev/ttyUSB1");
  
  // 设置消息回调函数来获取处理后的消息
  serial_receiver->set_message_callback([&processed_log](const serial_comm::SerialMessage& msg) {
    // 输出接收到的消息信息到控制台
    std::cout << "Received message ID: " << msg.id << std::endl;
    std::cout << "Quaternion: [" << msg.data.quaternion.w << ", " 
              << msg.data.quaternion.x << ", " 
              << msg.data.quaternion.y << ", " 
              << msg.data.quaternion.z << "]" << std::endl;
    std::cout << "Angular Velocity: [" << msg.data.angular_velocity.x << ", " 
              << msg.data.angular_velocity.y << ", " 
              << msg.data.angular_velocity.z << "]" << std::endl;
    std::cout << "Linear Acceleration: [" << msg.data.linear_acceleration.x << ", " 
              << msg.data.linear_acceleration.y << ", " 
              << msg.data.linear_acceleration.z << "]" << std::endl;
    std::cout << "CRC16: " << msg.crc16 << std::endl;
    std::cout << "----------------------------" << std::endl;
    
    // 同时写入文件以便后续分析
    processed_log << "ID: " << msg.id << std::endl;
    processed_log << "Quaternion: " << msg.data.quaternion.w << " " 
                  << msg.data.quaternion.x << " " 
                  << msg.data.quaternion.y << " " 
                  << msg.data.quaternion.z << std::endl;
    processed_log << "Angular Velocity: " << msg.data.angular_velocity.x << " " 
                  << msg.data.angular_velocity.y << " " 
                  << msg.data.angular_velocity.z << std::endl;
    processed_log << "Linear Acceleration: " << msg.data.linear_acceleration.x << " " 
                  << msg.data.linear_acceleration.y << " " 
                  << msg.data.linear_acceleration.z << std::endl;
    processed_log << "CRC16: " << msg.crc16 << std::endl;
    processed_log << "----------------------------" << std::endl;
  });
  
  serial_receiver->start();
  
  // 让程序持续运行一段时间以便收集数据
  std::cout << "Receiver running. Press Enter to stop..." << std::endl;
  std::cin.get();
  
  processed_log.close();
  return 0;
}