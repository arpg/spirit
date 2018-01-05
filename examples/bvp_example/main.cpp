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
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);



  spTrajectory traj(spworld.gui_,spworld.objects_);

  
  float x0 = 0;
  float y0 = 0;
  float yaw0 = 0;
  float v0 = 0;

  float thrust_u = 0;
  float turn_u = 0;
  float duration = 1;

  float x1 = 0;
  float y1 = 0;
  float yaw1 = 0;
  float v1 = 0;

  std::ifstream in_file;
  in_file.open ("temp_files/bvp_input.txt");
  in_file >> yaw0 >> x0 >> y0 >> v0 ;
  in_file >> thrust_u >> turn_u >> duration ;
  in_file >> yaw1 >> x1 >> y1 >> v1 ;
  in_file.close();

  std::cout << yaw0 << ", " << x0 << ", " << y0 << ", " << v0 << std::endl;
  std::cout << thrust_u << ", " << turn_u << ", " << duration << std::endl;
  std::cout << yaw1 << ", " << x1 << ", " << y1 << ", " << v1 << std::endl;

  spPose pose0(spPose::Identity());
  pose0.translate(spTranslation(x0,y0,0.06));
  Eigen::AngleAxisd rot0(yaw0,Eigen::Vector3d::UnitZ());
  pose0.rotate(rot0);
  traj.AddWaypoint(pose0,v0);

  spPose pose1(spPose::Identity());
  pose1.translate(spTranslation(x1,y1,0.06));
  // double angle = SP_PI/10;
  Eigen::AngleAxisd rot1(yaw1,Eigen::Vector3d::UnitZ());
  pose1.rotate(rot1);
  traj.AddWaypoint(pose1,v1);

  traj.IsLoop(false);

  spLocalPlanner localplanner(spworld.car_param,&spworld.gui_);

  spworld.gui_.Iterate(spworld.objects_);

  spCurve controls_curve(2,2);
 spCtrlPts2ord_2dof cntrl_cmd;
 cntrl_cmd.col(0) = Eigen::Vector2d(turn_u,thrust_u);
 cntrl_cmd.col(1) = Eigen::Vector2d(turn_u,thrust_u);
 cntrl_cmd.col(2) = Eigen::Vector2d(turn_u,thrust_u);
 // controls_curve.SetBezierControlPoints(cntrl_cmd);

  

  traj.SetTravelDuration(0,duration); // index of waypoint, time duration (seconds)
  traj.SetControls(0, cntrl_cmd);
  localplanner.SolveInitialPlan(traj,0);
  double final_cost = localplanner.SolveLocalPlan(traj,0,false); // traj, indx of waypoint, editWayPoint?
//  double sim_duration = 1;
//  double final_cost = localplanner.SolveLocalPlan((spCtrlPts2ord_2dof&)controls_curve.GetBezierControlPoints(),sim_duration,traj.Getz
  std::cout << "final cost: " << final_cost << std::endl;
  controls_curve.SetBezierControlPoints(traj.GetControls(0));
  std::cout << "bezier control points:\n"<<controls_curve.GetBezierControlPoints() << std::endl;
  
  std::shared_ptr<spStateSeries> statesSeries = traj.GetTrajectoryStateSeries(0);
  for(int ii = 0; ii < statesSeries->size(); ii++){
    std::shared_ptr<spState> state = statesSeries->at(ii);
    float x = state->pose.translation()[0];
    float y = state->pose.translation()[1];
    Eigen::Matrix3d rotmat = state->pose.rotation();
    float yaw = std::atan2(rotmat(1,0),rotmat(0,0));
    float v = state->linvel.norm();
    // std::cout << "yaw:" << yaw << std::endl;
    // std::cout << "x:" << x << std::endl;
    // std::cout << "y:" << y << std::endl;
    // std::cout << "v:" << v << std::endl;
    std::cout << yaw << " " << x << " " << y << " " << v << std::endl;
  }
  // while(1) {
  //   spworld.gui_.Iterate(spworld.objects_);

  //   // print states
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
  return 0;
}
