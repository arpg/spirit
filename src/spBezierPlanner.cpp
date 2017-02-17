#include <spirit/Planners/spBezierPlanner.h>

spBezierPlanner::spBezierPlanner(Gui* gui) : jac_gui_(gui){
  is_loop_ = false;
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0,0,0));
//  Eigen::AngleAxisd rot(M_PI/4+0.17355,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0;
  car_param.wheel_friction = 0.3;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 10;
  car_param.susp_stiffness = 100;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 2;
  car_param.steering_servo_upper_limit = SP_PI / 2;
//  jac_physics_;
//  jac_physics_.Create(PHY_BULLET);
  jac_car_handle = jac_objects_.CreateVehicle(car_param);
//  jac_physics_.AddObject(jac_objects_.GetObject(jac_car_handle));
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0,0,-0.5));
//  jac_physics_.AddObject(jac_objects_.GetObject(jac_objects_.CreateBox(ground,spBoxSize(10,10,1),0,spColor(0,1,0))));
  jac_gui_->AddObject(jac_objects_.GetObject(jac_car_handle));
}

spBezierPlanner::~spBezierPlanner(){

}

void spBezierPlanner::AddWaypoint(const spWaypoint& waypoint, unsigned int index) {
  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
  std::vector<std::shared_ptr<spCurve>>::iterator curve_it;
  waypoint_it = planpoint_vec_.begin();
  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
  curve_it = plancurve_vec_.begin();
  plancurve_vec_.insert(curve_it+index, new_curve);
  std::vector<bool>::iterator flag_it;
  flag_it = needs_curveupdate_vec_.begin();
  needs_curveupdate_vec_.insert(flag_it+index,true);
}

void spBezierPlanner::AddWaypoint(const spWaypoint& waypoint) {
  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
  planpoint_vec_.push_back(new_waypoint);
  plancurve_vec_.push_back(new_curve);
  needs_curveupdate_vec_.push_back(false);
//  std::cout << "plan point added to index: " << plancurve_vec_.size()-1 << std::endl;
}

const spWaypoint& spBezierPlanner::GetWaypoint(unsigned int index) {
  if(index>planpoint_vec_.size()-1) {
    SPERROREXIT("Requested index doesn't exist.");
  }
  return *planpoint_vec_[index];
}

void spBezierPlanner::UpdateWaypoint(const spWaypoint& planpoint,unsigned int index) {
//  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
//  std::shared_ptr<spBezierCurve> new_curve = std::make_shared<spBezierCurve>();
//  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
//  std::vector<std::shared_ptr<spBezierCurve>>::iterator curve_it;
//  waypoint_it = planpoint_vec_.begin();
//  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
//  curve_it = plancurve_vec_.begin();
//  plancurve_vec_.insert(curve_it+index, new_curve);
}

const spCurve& spBezierPlanner::GetCurve(unsigned int index) {

}

void spBezierPlanner::RemoveWaypoint(unsigned int index_in_plan) {

}

void spBezierPlanner::CalcJacobian(spPlannerJacobian& jacobian, const spCtrlPts3ord_2dof& cntrl_vars,unsigned int num_sim_steps,double sim_step_size, const spPose& init_pose, double fd_delta) {
/*
  spAWSDCar& car = (spAWSDCar&) jac_objects_.GetObject(jac_car_handle);
  spCurve control_curve(3,2);
  // 8+1 simulations required to fill the jacobian
  control_curve.SetBezierControlPoints(cntrl_vars);
  spPointXd sample_control(2);
  car.SetPose(init_pose);
  car.SetClampToSurfaceFlag();
  // simulate final pose of vehicle with current control_vars
  for(int ii=1;ii<=num_sim_steps;ii++) {
    control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
    car.SetFrontSteeringAngle(sample_control[0]);
    car.SetEngineTorque(sample_control[1]);
//    jac_physics_.Iterate(jac_objects_,sim_step_size);
    jac_gui_->Iterate(jac_objects_);
    spGeneralTools::Delay_ms(100);
  }
  spStateVec end_state = car.GetStateVecor();
  // now do the same thing with fd_delta applied to spCurve
  control_curve.SetBezierControlPoints(cntrl_vars);
  for(int jj=0;jj<8;jj++) {
    // set state of vehicle
    car.SetPose(init_pose);
    car.SetClampToSurfaceFlag();
    // perturb a control signal
    control_curve.PerturbControlPoint(jj,fd_delta);
    spPointXd sample_control(2);
    // simulate final pose of vehicle with perturbed control_vars
    for(int ii=1;ii<=num_sim_steps;ii++) {
      control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineTorque(sample_control[1]);
//      jac_physics_.Iterate(jac_objects_,sim_step_size);
    }
    control_curve.RemoveLastPerturbation();
    spStateVec perturbed_state_delta = car.GetStateVecor();
    // find forward finite difference value and put in jacobian
    jacobian.col(jj) = (perturbed_state_delta-end_state)/fd_delta;
  }
  std::cout << "jacobian is : \n" << jacobian << std::endl;
*/
}


int spBezierPlanner::GetNumWaypoints() {
  return planpoint_vec_.size();
}

void spBezierPlanner::IsLoop(bool is_loop) {
  is_loop_ = is_loop;
}

void spBezierPlanner::UpdateCurves() {
  for(int ii=0;ii<planpoint_vec_.size();ii++) {
    if(needs_curveupdate_vec_[ii]) {
      // solveBVP of that section
      std::cout << "needs update -> " << ii << std::endl;
    }
  }
}

void spBezierPlanner::SolveBVP() {

}
