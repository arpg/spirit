#include <spirit/spirit.h>

int main(int argc, char** argv) {

  // Create world setting object
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  // create the world object
  spirit spworld(settings_obj);

  // create a car with default values at car_param object
//  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
//  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
//  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);

  // create a flat ground with a box object
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_->CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_->GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_->GetObject(gnd_handle)).SetFriction(1);



  spTrajectory traj(spworld.gui_, spworld.objects_);


  spPose pose0(spPose::Identity());
  pose0.translate(spTranslation(0,0,0.06));
  Eigen::AngleAxisd rot0(0,Eigen::Vector3d::UnitZ());
  pose0.rotate(rot0);
  traj.AddWaypoint(pose0,4);

  spPose pose1(spPose::Identity());
  pose1.translate(spTranslation(0.5,2,0.06));
  double angle = SP_PI/10;
  Eigen::AngleAxisd rot1(angle,Eigen::Vector3d::UnitZ());
  pose1.rotate(rot1);
  traj.AddWaypoint(pose1,4);

  traj.IsLoop(false);

  spLocalPlanner localplanner(spworld.car_param,false,&spworld.gui_);

  spworld.gui_.Iterate(spworld.objects_);

  spCurve controls_curve(2,2);
//  spCtrlPts2ord_2dof cntrl_cmd;
//  cntrl_cmd.col(0) = Eigen::Vector2d(0,0);
//  cntrl_cmd.col(1) = Eigen::Vector2d(0,20);
//  cntrl_cmd.col(2) = Eigen::Vector2d(0,50);
//  controls_curve.SetBezierControlPoints(cntrl_cmd);

  traj.SetTravelDuration(0,1);
  localplanner.SolveInitialPlan(traj,0);
  double final_cost = localplanner.SolveLocalPlan(traj,0);
//  double sim_duration = 1;
//  double final_cost = localplanner.SolveLocalPlan((spCtrlPts2ord_2dof&)controls_curve.GetBezierControlPoints(),sim_duration,traj.Getz
  std::cout << "final cost: " << final_cost << std::endl;

  controls_curve.SetBezierControlPoints(traj.GetControls(0));
  std::cout << "bezier control points:\n"<<controls_curve.GetBezierControlPoints() << std::endl;

  while(1) {
    spworld.gui_.Iterate(spworld.objects_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
