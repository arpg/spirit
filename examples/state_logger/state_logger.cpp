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
#include <iostream>
#include <fstream>
#include <mutex>


struct OptiState{
  spPose prev_pose;
  spPose pose;
  spLinVel linvel;
  spRotVel rotvel;
  double prev_dev_time;
  bool initialized = false;
} optistate_;

std::mutex filemutex_;
spTimestamp start_timestamp_ = spGeneralTools::Tick();
std::ofstream logfile_;
hal::CarCommandMsg commandMSG;
double init_sysdev_diff_;

void optitrack_pose_handler(hal::PoseMsg& PoseData) {

  // get x-y translation
  optistate_.pose = spPose::Identity();
  optistate_.pose.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0));

  // get yaw rotation
  spRotation full_rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  optistate_.pose.rotate(full_rot);
  Eigen::AngleAxisd curr_rot(optistate_.pose.rotation());
  double curr_yaw = curr_rot.angle()*curr_rot.axis()[2];

  // Calculate Velocities
  if(optistate_.initialized){
    double inv_time_dif = 1/(PoseData.device_time()-optistate_.prev_dev_time);
    optistate_.linvel[0] = inv_time_dif*((optistate_.pose.translation()[0])-(optistate_.prev_pose.translation()[0]));
    optistate_.linvel[1] = inv_time_dif*((optistate_.pose.translation()[1])-(optistate_.prev_pose.translation()[1]));
    optistate_.linvel[2] = 0;
    optistate_.rotvel[0] = 0;
    optistate_.rotvel[1] = 0;
    Eigen::AngleAxisd prev_rot(optistate_.prev_pose.rotation());
    double yaw_diff = curr_yaw-prev_rot.angle()*prev_rot.axis()[2];
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

  // copy to prev pose and dev_time
  optistate_.prev_pose = optistate_.pose;
  optistate_.prev_dev_time = PoseData.device_time();
  optistate_.initialized = true;

  // write to file
  filemutex_.lock();
  logfile_ << 2 << "," << spGeneralTools::Tock_us(start_timestamp_)*1e-6 << ","
           << optistate_.pose.translation()[0] << ","
           << optistate_.pose.translation()[1] << ","
           << curr_yaw << ","
           << optistate_.linvel[0] << ","
           << optistate_.linvel[1] << ","
           << optistate_.rotvel[2] << "\n";
  logfile_.flush();
  filemutex_.unlock();
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  commandMSG.set_steering_angle(-_msg.axes().data(0));
  commandMSG.set_throttle_percent(_msg.axes().data(3)*40);
}

void CarSensorCallback(hal::CarStateMsg msg) {
    std::cout << " car state -> "
              << msg.wheel_speed_fl() << ", "
              << msg.wheel_speed_fr() << ", "
              << msg.wheel_speed_rl() << ", "
              << msg.wheel_speed_rr() << ", "
              << msg.steer_angle()
              << std::endl;

    filemutex_.lock();
    logfile_ << 1 << "," << spGeneralTools::Tock_us(start_timestamp_)*1e-6 << ","
             << msg.wheel_speed_fl() << ","
             << msg.wheel_speed_fr() << ","
             << msg.wheel_speed_rl() << ","
             << msg.wheel_speed_rr() << ","
             << msg.steer_angle() << "\n";
    logfile_.flush();
    filemutex_.unlock();
}

int main(int argc, char** argv) {

  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
//  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");
//  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // Connect to Optitrack system
  hal::Posys optitrack("vicon://tracker:[dummy]");
  optitrack.RegisterPosysDataCallback(&optitrack_pose_handler);

  logfile_.open("log.csv");

  while(1){
//    ninja_car.UpdateCarCommand(commandMSG);

    filemutex_.lock();
    logfile_ << 0 << "," << spGeneralTools::Tock_us(start_timestamp_)*1e-6 << "," <<commandMSG.steering_angle() << "," << commandMSG.throttle_percent() << "\n";
    logfile_.flush();
    filemutex_.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  logfile_.close();
  return 0;
}
