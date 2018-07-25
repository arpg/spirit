#include <thread>
#include <spirit/spirit.h>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <HAL/Posys/PosysDevice.h>

struct OptiState{
  spPose prev_pose;
  spPose pose;
  spLinVel linvel;
  spRotVel rotvel;
  bool initialized = false;
} optistate_;

void optitrack_pose_handler(hal::PoseMsg& PoseData) {
  // get x-y translation
  optistate_.pose = spPose::Identity();
  optistate_.pose.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0));

  // get yaw rotation
  spRotation full_rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  Eigen::AngleAxisd yaw_rot(full_rot.EulerAngles(0,1,2)[3],Eigen::Vector3d::UnitZ());
  optistate_.pose.rotate(yaw_rot);

  // Calculate Velocities
  if(optistate_.initialized){
    double inv_time_dif = 1/(PoseData.device_time()-prev_pose_.device_time());
    optistate_.linvel[0] = inv_time_dif*(optistate_.pose.translation()[0]-optistate_.prev_pose.translation()[0]);
    optistate_.linvel[1] = inv_time_dif*(optistate_.pose.translation()[1]-optistate_.prev_pose.translation()[1]);
    optistate_.linvel[2] = 0;
    prev_pose_.rotvel[0] = 0;
    prev_pose_.rotvel[1] = 0;
    double yaw_diff = (optistate_.pose.rotation().EulerAngles(0,1,2)[2]-optistate_.prev_pose.rotation().EulerAngles(0,1,2)[2]);
    if(yaw_diff>SP_PI){
      yaw_diff = 2*SP_PI-yaw_diff;
    } else if(yaw_diff<-SP_PI){
      yaw_diff = -2*SP_PI-yaw_diff;
    }
    prev_pose_.rotvel[2] = inv_time_dif*yaw_diff;
  } else {
    linvel_ = spLinVel(0,0,0);
    rotvel_ = spRotVel(0,0,0);
  }

  // system time vs Device time diff
  optistate_.prev_pose = optistate_.pose;

  optistate_.initialized = true;
}

int main(int argc, char** argv) {

  ////////////////////////////////////////////////////////////
  hal::Posys vicon("vicon://tracker:[dummy]");
  vicon.RegisterPosysDataCallback(&optitrack_pose_handler);
  ////////////////////////////////////////////////////////////

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  return 0;
}
