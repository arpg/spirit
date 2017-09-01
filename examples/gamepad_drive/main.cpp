#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>


hal::CarCommandMsg commandMSG;
spPose posys_;

void Posys_Handler(hal::PoseMsg& PoseData) {
  std::cout << "Posys Id: " << PoseData.id() << ". Data: ";
  for (int ii = 0; ii < PoseData.pose().data_size(); ++ii) {
    std::cout << PoseData.pose().data(ii) << " ";
  }
  std::cout << std::endl;
  posys_ = spPose::Identity();
  posys_.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),PoseData.pose().data(2)));
  spRotation rot(PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5),PoseData.pose().data(6));
  posys_.rotate(rot);
}

void GamepadCallback(hal::GamepadMsg& _msg) {
//  std::cout << "-> "
//            << _msg.axes().data(0) << ", "
//            << _msg.axes().data(1) << ", "
//            << _msg.axes().data(2) << ", "
//            << _msg.axes().data(3) << ", "
//            << _msg.axes().data(4) << ", "
//            << _msg.axes().data(5) << ", "
//            << _msg.axes().data(6) << ", "
//            << _msg.axes().data(7) << ", "
//            << _msg.axes().data(8) << ", "
//            << _msg.axes().data(9) << ", "
//            << _msg.axes().data(10) << ", "
//            << _msg.axes().data(11) << ", "
//            << _msg.axes().data(12) << ", "
//            << _msg.axes().data(13) << ", "
//            << _msg.axes().data(14) << " -  "
//            << _msg.buttons().data(0) << ","
//            << _msg.buttons().data(1) << ","
//            << _msg.buttons().data(2) << ","
//            << _msg.buttons().data(3) << ","
//            << _msg.buttons().data(4) << ","
//            << _msg.buttons().data(5) << ","
//            << _msg.buttons().data(6) << ","
//            << _msg.buttons().data(7) << ","
//            << _msg.buttons().data(8) << ","
//            << _msg.buttons().data(9) << ","
//            << _msg.buttons().data(10) << ","
//            << _msg.buttons().data(11) << ","
//            << std::endl;
  // update transmit command with gamepad data
  commandMSG.set_steering_angle(_msg.axes().data(0));
  commandMSG.set_throttle_percent(_msg.axes().data(0)*20);
}

void CarSensorCallback(hal::CarStateMsg msg) {
//    std::cout << "-> "
//              << msg.swing_angle_fl() << ", "
//              << msg.swing_angle_fr() << ", "
//              << msg.swing_angle_rl() << ", "
//              << msg.swing_angle_rr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_fr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_rl() << ", "
//              << msg.wheel_speed_rr() << ", "
//              << msg.steer_angle() << ", "
//              << std::endl;
}


int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
//  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00002014A]//");

//  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  //////////////////////////////
  hal::Posys posys_("vicon:://192.168.20.100:[Ninja1]");
//  posys_.RegisterPosysDataCallback(std::bind(&Posys_Handler,this, _1));
  posys_.RegisterPosysDataCallback(&Posys_Handler);
  /////////////////////////////
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);
  spirit spworld(settings_obj);

  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);
  // create a flat ground
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));


  while(1) {
//    ninja_car.UpdateCarCommand(commandMSG);
    car.SetPose(posys_);
    spworld.gui_.Iterate(spworld.objects_);
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
