#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include "clf.h"

hal::CarCommandMsg commandMSG;
double gamepad_steering = 0;
double gamepad_throttle = 0;
bool flag_auto = false;

spPose vicon_pose;
spPose vicon_prev_pose;
spLinVel ninja_linvel;
spRotVel ninja_rotvel;
spTimestamp vicon_t0;
double vicon_time_elapsed = 0;
bool init_state_flg = true;

void vicon_poseHandler(hal::PoseMsg& PoseData) {
    vicon_time_elapsed = spGeneralTools::Tock_ms(vicon_t0)/1000.0;
if(vicon_time_elapsed == 0)   return;
    vicon_t0 = spGeneralTools::Tick();
    vicon_pose = spPose::Identity();
    vicon_pose.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0.07/*PoseData.pose().data(2)*/));
    spRotation rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
    Eigen::AngleAxisd tracker_rot(-SP_PI_HALF,Eigen::Vector3d::UnitZ());
    vicon_pose.rotate(rot);
    vicon_pose.rotate(tracker_rot);


    if(init_state_flg) {
      ninja_linvel = spLinVel::Zero();
      ninja_rotvel = spRotVel::Zero();
      init_state_flg = false;
    } else {
        spPose diff = vicon_prev_pose.inverse()*vicon_pose;
        ninja_linvel = (vicon_pose.translation()-vicon_prev_pose.translation())/vicon_time_elapsed;
        ninja_linvel[2] = 0;
        Eigen::AngleAxisd angleaxis(diff.rotation());
        Eigen::Vector3d rotvec(angleaxis.angle()*angleaxis.axis());
        ninja_rotvel = rotvec/vicon_time_elapsed;
        ninja_rotvel[0] = 0;
        ninja_rotvel[1] = 0;
    }
   vicon_prev_pose = vicon_pose;
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  gamepad_steering = -_msg.axes().data(0);
  gamepad_throttle = _msg.axes().data(4)*30;
  if(_msg.buttons().data(5)){
      flag_auto = true;
  } else {
      flag_auto = false;
  }
}

int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");

  //ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  //////////////////////////////
  hal::Posys vicon("vicon://tracker:[ninja]");
  vicon.RegisterPosysDataCallback(&vicon_poseHandler);
  /////////////////////////////

//  double x0 = -1;
//  double y0 = -1;
//  double th0 = -1.6268;
//  double v0 = 1.43;
  double p0 = 0;

  double torque0 = 0; // range: -50, 50
  double turn0 = 1.2667; //SP_PI_QUART; // range: -pi/4, pi/4
  double dp0 = 0;

//  std::ifstream in_file;
//  in_file.open ("temp_files/sim_input.txt");
//  in_file >> th0 >> x0 >> y0 >> v0 >> torque0 >> turn0;
//  in_file.close();

//  double v0_x = -v0*sin(th0);
//  double v0_y = v0*cos(th0);

bool prev_flag = false;

  while(1){

    double p_t = p0;
    double u_1_prev = torque0;
    double u_2_prev = turn0;
    double u_3_prev = dp0;

    double tau = 0.01;

    int seg_prev = 0;
    int cnt = 0;

    while(p_t < 64) {
      Eigen::Matrix3d rotmat = vicon_pose.rotation();
      double x_t = vicon_pose.translation()[0];
      double y_t = vicon_pose.translation()[1];
      double th_t = std::atan2(rotmat(1,0),rotmat(0,0));
      th_t -= SP_PI_HALF;
      double v_t = 1.57;//ninja_linvel.norm();
      p_t += (1+u_3_prev)*tau;

      Input feedback = K(th_t, x_t, y_t, v_t, p_t, seg_prev, u_1_prev, u_2_prev, u_3_prev);

      double u_1 = feedback.u_1;
      double u_2 = feedback.u_2;
      double u_3 = feedback.u_3;
      int seg = feedback.seg;

      u_1_prev = u_1;
      u_2_prev = u_2;
      u_3_prev = u_3;
      seg_prev = seg;

      //double torque = (u_1-0.3124)/13908;
      //torque += 0.0002;
      double torque = u_1 *3  + 17;
      double turn = atan(u_2);

      if (cnt>1) {
        std::cout << "log : " << x_t << "\t , \t" << y_t << "\t , \t"<< th_t << "\t , \t"  << v_t << "\t , \t" << p_t<< std::endl;
        std::cout << "inp : " << u_1 << "\t , \t" << u_2 << "\t , \t"<< u_3  << std::endl;
        std::cout << "rad :" << std::sqrt(x_t*x_t+y_t*y_t) << std::endl;
        cnt = 0;
       } else {
	 cnt++;
       }

      // Apply signals to the ninja car
      if(!flag_auto) {
        if(turn > SP_PI_QUART)  turn = SP_PI_QUART;
        if(turn < -SP_PI_QUART)  turn = -SP_PI_QUART;
        if(torque > 30)  torque = 30;
        if(torque < -30)  torque = -30;
        commandMSG.set_steering_angle(-turn);
        commandMSG.set_throttle_percent(-torque);
      } else {
        commandMSG.set_steering_angle(gamepad_steering);
        commandMSG.set_throttle_percent(gamepad_throttle);
      }
      //commandMSG.set_throttle_percent(gamepad_throttle);
      ninja_car.UpdateCarCommand(commandMSG);
      if((flag_auto==true) && (prev_flag==false))    p_t = 0;
      prev_flag = flag_auto;
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(tau*1000*(1))));
    }
  }
  return 0;
}
