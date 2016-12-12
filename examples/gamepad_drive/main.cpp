#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>

hal::CarCommandMsg commandMSG;

void GamepadCallback(hal::GamepadMsg& _msg) {
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
  // update transmit command with gamepad data
  commandMSG.set_steering_angle(_msg.axes().data(0));
  commandMSG.set_throttle_percent(_msg.axes().data(0)*20);
}

void CarSensorCallback(hal::CarStateMsg msg) {
    std::cout << "-> "
              << msg.swing_angle_fl() << ", "
//              << msg.swing_angle_fr() << ", "
//              << msg.swing_angle_rl() << ", "
//              << msg.swing_angle_rr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_fr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_rl() << ", "
//              << msg.wheel_speed_rr() << ", "
//              << msg.steer_angle() << ", "
              << std::endl;
}


int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00002014A]//");

  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);

  while(1) {
    ninja_car.UpdateCarCommand(commandMSG);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}


//volatile sig_atomic_t flag = 0;
//void my_function(int sig){ // can be called asynchronously
//  flag = 1; // set flag
//  std::cout << "ctrl-c called" << std::endl;
//}
//  signal(SIGINT, my_function);
//    if(flag) {
//      std::cout << "now breaking" << std::endl;
//      break;
//    }
