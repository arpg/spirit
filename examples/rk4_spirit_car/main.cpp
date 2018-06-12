#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

int main(int argc, char** argv) {

  spState state;
  state.pose = spPose::Identity();
  Eigen::AngleAxisd rot1(-SP_PI/4,Eigen::Vector3d::UnitZ());
  state.pose.rotate(rot1);
  state.linvel = spLinVel(0,0,0);
  state.rotvel = spRotVel(0,0,0);
  state.wheel_speeds = spWheelSpeedVec(0,0,0,0);

  std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

  spVehicleConstructionInfo info;

  spCtrlPts2ord_2dof inputcmd_curve;
  inputcmd_curve.col(0) = Eigen::Vector2d(0.7,1);
  inputcmd_curve.col(1) = Eigen::Vector2d(0.7,1);
  inputcmd_curve.col(2) = Eigen::Vector2d(0.7,1);

  CarSimFunctorRK4 mysim(info,state);
  for(int ii=0;ii<50000;ii++){
    mysim(0,1,0.01,inputcmd_curve,0,0,nullptr,state_ptr);
    std::cout << mysim.GetState().pose.translation()[0] << "," << mysim.GetState().pose.translation()[1] << std::endl;
  }

  std::cout << "Done ... !" << std::endl;

  return 0;
}
