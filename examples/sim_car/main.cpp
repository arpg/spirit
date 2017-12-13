#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>


//hal::CarCommandMsg commandMSG;

//void GamepadCallback(hal::GamepadMsg& _msg) {
//  commandMSG.set_steering_angle(_msg.axes().data(0));
//  commandMSG.set_throttle_percent(_msg.axes().data(5)*20);
//}

int main(int argc, char** argv) {
  // connect to a gamepad
//  hal::Gamepad gamepad("gamepad:/");
//  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Initialize commands
//  commandMSG.set_steering_angle(0);
//  commandMSG.set_throttle_percent(0);

  // Create world setting object
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  // create the world object
  spirit spworld(settings_obj);

  // create a car with default values at car_param object
  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);

  // create a flat ground with a box object
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);


  while(1) {
//    car.SetEngineMaxVel(commandMSG.throttle_percent());
//    car.SetFrontSteeringAngle(commandMSG.steering_angle());

    // set some constant values for engine velocity and front steering angle
    car.SetEngineMaxVel(20);
    car.SetFrontSteeringAngle(SP_PI_QUART);

    // step physics simulation for 0.01 seconds
    spworld.objects_.StepPhySimulation(0.01);

    // iterate gui to update object status
    spworld.gui_.Iterate(spworld.objects_);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
