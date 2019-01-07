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
      yaw_diff = -2*SP_PI+yaw_diff;
    } else if(yaw_diff<-SP_PI){
      yaw_diff = 2*SP_PI+yaw_diff;
    }
    //optistate_.rotvel[2] = inlinv_time_dif*yaw_diff;
    double yaw_rate = inv_time_dif*yaw_diff;

    // filter yaw rate with averaging filter
    const unsigned int buf_size = 10;
    static double buf[buf_size];
    static unsigned int last_index_ptr = 0;

    buf[last_index_ptr] = yaw_rate;
    if(last_index_ptr==buf_size-1){
        last_index_ptr = 0;
    } else {
        last_index_ptr++;
    }
    double sum = 0;
    for(int ii=0; ii<buf_size; ii++){
        sum += buf[ii];
    }
    optistate_.rotvel[2] = sum/buf_size;

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
  commandMSG.set_throttle_percent(_msg.axes().data(4)*40);
}

void CarSensorCallback(hal::CarStateMsg msg) {
//    std::cout << " car state -> "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_fr() << ", "
//              << msg.wheel_speed_rl() << ", "
//              << msg.wheel_speed_rr() << ", "
//              << msg.steer_angle()
//              << std::endl;

    // filter noisy data - I get sometimes very large or very small numbers which could be due to error in the ECU firmware.
    if((std::abs(msg.wheel_speed_fl())>300)||(std::abs(msg.wheel_speed_fr())>300)||(std::abs(msg.wheel_speed_rl())>300)||(std::abs(msg.wheel_speed_rr())>300)){
        return;
    }

    // filter all data with running average
    const unsigned int buf_size = 10;
    static double buf0[buf_size];
    static double buf1[buf_size];
    static double buf2[buf_size];
    static double buf3[buf_size];
    static unsigned int last_index_ptr = 0;

    // BUG: wheel order is messed up, change this later to following commented code as well as the hack on LogParser.h
//    buf0[last_index_ptr] = msg.wheel_speed_fl();
//    buf1[last_index_ptr] = msg.wheel_speed_fr();
//    buf2[last_index_ptr] = msg.wheel_speed_rl();
//    buf3[last_index_ptr] = msg.wheel_speed_rr();
    buf0[last_index_ptr] = msg.wheel_speed_fl();
    buf1[last_index_ptr] = msg.wheel_speed_rl();
    buf2[last_index_ptr] = msg.wheel_speed_rr();
    buf3[last_index_ptr] = msg.wheel_speed_fr();

    if(last_index_ptr==buf_size-1){
        last_index_ptr = 0;
    } else {
        last_index_ptr++;
    }
    double sum0 = 0;
    double sum1 = 0;
    double sum2 = 0;
    double sum3 = 0;
    for(int ii=0; ii<buf_size; ii++){
        sum0 += buf0[ii];
        sum1 += buf1[ii];
        sum2 += buf2[ii];
        sum3 += buf3[ii];
    }
    double ws0 = sum0/buf_size;
    double ws1 = sum1/buf_size;
    double ws2 = sum2/buf_size;
    double ws3 = sum3/buf_size;

    filemutex_.lock();
    logfile_ << 1 << "," << spGeneralTools::Tock_us(start_timestamp_)*1e-6 << ","
    // BUG: wheel order is messed up, change this later to following commented code as well as the hack on LogParser.h
             << ws0 << ","
             << ws1 << ","
             << -ws2 << ","
             << -ws3 << ","
             << msg.steer_angle() << "\n";
    logfile_.flush();
    filemutex_.unlock();
}

int main(int argc, char** argv) {

  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");
  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // Connect to Optitrack system
  hal::Posys optitrack("vicon://tracker:ninja");
  optitrack.RegisterPosysDataCallback(&optitrack_pose_handler);

  logfile_.open("log.csv");

  while(1){
    ninja_car.UpdateCarCommand(commandMSG);

    filemutex_.lock();
    logfile_ << 0 << "," << spGeneralTools::Tock_us(start_timestamp_)*1e-6 << "," << -commandMSG.steering_angle() << "," << -commandMSG.throttle_percent()/40.0 << "\n";
    logfile_.flush();
    filemutex_.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  logfile_.close();
  return 0;
}
