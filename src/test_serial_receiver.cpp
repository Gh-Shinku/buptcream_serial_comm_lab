#include "serial_comm/serial_receiver.hpp"
#include <memory>

int main() {
  auto serial_receiver = std::make_unique<serial_comm::SerialReceiver>("/dev/ttyUSB1");
  serial_receiver->start();
  return 0;
}