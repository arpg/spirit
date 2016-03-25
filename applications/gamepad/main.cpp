#include "HAL/Gamepad/GamepadDevice.h"
#include "HAL/Gamepad.pb.h"

void func(hal::GamepadMsg& _msg) {
  //    std::cout << "func has been called .........." << std::endl;
  std::cout << "-> "
            << _msg.axes().data(0) << ", "
            << _msg.axes().data(1) << ", "
            << _msg.axes().data(2) << ", "
            << _msg.axes().data(3) << ", "
            << _msg.axes().data(4) << ", "
            << _msg.axes().data(5) << ", "
            << _msg.axes().data(6) << ", "
            << _msg.axes().data(7) << ", "
            << _msg.axes().data(8) << ", "
            << _msg.axes().data(9) << ", "
            << _msg.axes().data(10) << ", "
            << _msg.axes().data(11) << ", "
            << _msg.axes().data(12) << ", "
            << _msg.axes().data(13) << ", "
            << _msg.axes().data(14) << " -  "
            << _msg.buttons().data(0) << ","
            << _msg.buttons().data(1) << ","
            << _msg.buttons().data(2) << ","
            << _msg.buttons().data(3) << ","
            << _msg.buttons().data(4) << ","
            << _msg.buttons().data(5) << ","
            << _msg.buttons().data(6) << ","
            << _msg.buttons().data(7) << ","
            << _msg.buttons().data(8) << ","
            << _msg.buttons().data(9) << ","
            << _msg.buttons().data(10) << ","
            << _msg.buttons().data(11) << ","
            << std::endl;
}

int main(int argc, char** argv) {
  hal::Gamepad _gamepad("gamepad:/");
  std::cout << "object created ..." << std::endl;
  _gamepad.RegisterGamepadDataCallback(&func);
  std::cout << "callback registered ... " << std::endl;
  std::cout << "starting while loop" << std::endl;
  while (1)
    ;
  std::cout << "end of program." << std::endl;
  return 0;
}
