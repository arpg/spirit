#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

int main(int argc, char** argv) {

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


  spState state;
  state.pose = spPose::Identity();
  state.pose.translate(spTranslation(2,1,0));
  Eigen::AngleAxisd rot1(0/*SP_PI/10*/,Eigen::Vector3d::UnitZ());
  state.pose.rotate(rot1);
  state.linvel = spLinVel(0,0,0);
  state.rotvel = spRotVel(0,0,0);
  state.wheel_speeds = spWheelSpeedVec(0,0,0,0);

  car.SetState(state);

  std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

  spVehicleConstructionInfo info;

  spCtrlPts2ord_2dof inputcmd_curve;
  inputcmd_curve.col(0) = Eigen::Vector2d(0.7,1);
  inputcmd_curve.col(1) = Eigen::Vector2d(0.7,1);
  inputcmd_curve.col(2) = Eigen::Vector2d(0.7,1);

  car.SetFrontSteeringAngle(0.7);
  spworld.gui_.Iterate(spworld.objects_);
//while(1);
  CarSimFunctorRK4 mysim(info,state);
  for(int ii=0;ii<1000;ii++){
    mysim(0,1,0.1,inputcmd_curve,0,0,nullptr,state_ptr);
    car.SetState(mysim.GetState());
    spworld.gui_.Iterate(spworld.objects_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << mysim.GetState().linvel[0] << "," << mysim.GetState().linvel[1] << std::endl;
  }
  while(1){
    spworld.gui_.Iterate(spworld.objects_);
  }
  std::cout << "Done ... !" << std::endl;

  return 0;
}
