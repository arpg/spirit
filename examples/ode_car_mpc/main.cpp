#include <thread>
#include <spirit/spirit.h>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>

struct OptiState{
  spPose prev_pose;
  spPose pose;
  spLinVel linvel;
  spRotVel rotvel;
  double prev_dev_time;
  bool initialized = false;
} optistate_;

struct GamepadState{
  double steering = 0;
  double acceleration = 0;
} gamepadstate_;

void optitrack_pose_handler(hal::PoseMsg& PoseData) {
  // get x-y translation
  optistate_.pose = spPose::Identity();
  optistate_.pose.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0));

  // get yaw rotation
  spRotation full_rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  Eigen::AngleAxisd yaw_rot(full_rot.toRotationMatrix().eulerAngles(0,1,2)[3],Eigen::Vector3d::UnitZ());
  optistate_.pose.rotate(yaw_rot);

  // Calculate Velocities
  if(optistate_.initialized){
    double inv_time_dif = 1/(PoseData.device_time()-optistate_.prev_dev_time);
    optistate_.linvel[0] = inv_time_dif*(optistate_.pose.translation()[0]-optistate_.prev_pose.translation()[0]);
    optistate_.linvel[1] = inv_time_dif*(optistate_.pose.translation()[1]-optistate_.prev_pose.translation()[1]);
    optistate_.linvel[2] = 0;
    optistate_.rotvel[0] = 0;
    optistate_.rotvel[1] = 0;
    double yaw_diff = (optistate_.pose.rotation().eulerAngles(0,1,2)[2]-optistate_.prev_pose.rotation().eulerAngles(0,1,2)[2]);
    if(yaw_diff>SP_PI){
      yaw_diff = 2*SP_PI-yaw_diff;
    } else if(yaw_diff<-SP_PI){
      yaw_diff = -2*SP_PI-yaw_diff;
    }
    optistate_.rotvel[2] = inv_time_dif*yaw_diff;
  } else {
    optistate_.linvel = spLinVel(0,0,0);
    optistate_.rotvel = spRotVel(0,0,0);
  }

  // system time vs Device time diff
  optistate_.prev_pose = optistate_.pose;
  optistate_.prev_dev_time = PoseData.device_time();

  optistate_.initialized = true;
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  gamepadstate_.steering = -_msg.axes().data(0);
  gamepadstate_.acceleration = _msg.axes().data(4)*40;
}

void CarSensorCallback(hal::CarStateMsg msg) {
    std::cout << " car state -> "
              << msg.wheel_speed_fl() << ", "
              << msg.wheel_speed_fr() << ", "
              << msg.wheel_speed_fl() << ", "
              << msg.wheel_speed_rl() << ", "
              << msg.steer_angle() << ", "
              << std::endl;
}

int main(int argc, char** argv) {

  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");
  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // Connect to Optitrack system
  hal::Posys vicon("vicon://tracker:[dummy]");
  vicon.RegisterPosysDataCallback(&optitrack_pose_handler);

  while(1){
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
