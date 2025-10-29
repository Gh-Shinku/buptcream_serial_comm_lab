cmake_minimum_required(VERSION 3.22)
project(serial_comm)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_DIR build)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

set(FINISH_RECEIVER ON)
set(TEST_RECEIVER ON)
set(FINISH_PUBLISHER ON)

add_compile_options(-Wall -Wpedantic -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  fmt::fmt
)

set(ROS_DEPS
  rclcpp
  # sensor_msgs::sensor_msgs  # 移除这一行
)

include_directories(include)

add_executable(serial_sender src/serial_sender.cpp)
target_link_libraries(serial_sender PRIVATE ${THIRD_DEPS})

if(TEST_RECEIVER AND NOT FINISH_RECEIVER)
  message(WARNING "TEST_RECEIVER requires FINISH_RECEIVER. Disabling TEST_RECEIVER.")
  set(TEST_RECEIVER ON)
endif()

if(FINISH_PUBLISHER AND NOT FINISH_RECEIVER)
  message(WARNING "FINISH_PUBLISHER requires FINISH_RECEIVER. Disabling FINISH_PUBLISHER.")
  set(FINISH_PUBLISHER ON)
endif()

if(FINISH_RECEIVER)
  add_library(lib_serial_receiver STATIC src/serial_receiver.cpp)
  target_link_libraries(lib_serial_receiver PUBLIC ${THIRD_DEPS})
endif()

if(TEST_RECEIVER)
  add_executable(test_serial_receiver src/test_serial_receiver.cpp)
  target_link_libraries(test_serial_receiver PRIVATE lib_serial_receiver)
endif()

if(FINISH_PUBLISHER)
  add_executable(serial_relay_node src/msg_publisher.cpp)
  
  # 移除PRIVATE关键字，使用普通签名
  target_link_libraries(serial_relay_node
    lib_serial_receiver
    ${THIRD_DEPS}
  )
  
  # 使用ament_target_dependencies添加ROS 2依赖
  ament_target_dependencies(serial_relay_node
    rclcpp
    sensor_msgs
  )
  
  # 保留包含目录设置
  target_include_directories(serial_relay_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  )
  
  # 确保在安装时也能正确链接
  install(TARGETS serial_relay_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()