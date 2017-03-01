#include <spirit/spirit.h>
#include <spirit/CarSimFunctor.h>
//#include <iomanip>
#include <spirit/Planners/spTrajectory.h>
#include <spirit/VehicleCeresCostFunc.h>

spirit::spirit(spSettings& user_settings) {
  user_settings_ = user_settings;
}

spirit::~spirit() {}

void spirit::Create() {
  // create gui object if requested
  if (user_settings_.GetGuiType() != spGuiType::GUI_NONE) {
    gui_.Create(user_settings_.GetGuiType());
  }
  // add physics world
  if (user_settings_.GetPhysicsEngineType() != spPhyEngineType::PHY_NONE) {
//    physics_.Create(user_settings_.GetPhysicsEngineType());
  }
}

bool spirit::ShouldRun() {
  if (gui_.ShouldQuit()) {
    return false;
  } else {
    return true;
  }
}

void spirit::CheckKeyboardAction() { gui_.CheckKeyboardAction(); }

void spirit::SenarioTrajectoryTest() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.07));
  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
  car_param.pose.rotate(rot1);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.6;
  car_param.wheel_friction = 0.3;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));

  spTrajectory traj(gui_,objects_);
  // put waypoints on a elliptical path
  double a = 3;
  double b = 2;
  for(int ii=0; ii<10; ii++) {
    // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
    double theta = ii*(2*SP_PI)/10;
    double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
    double x = r*cos(theta);
    double y = r*sin(theta);
    // slope of the line is
    double m = -(x*b*b)/(y*a*a);
    double angle = atan2(-(x*b*b),(y*a*a));
    spPose pose(spPose::Identity());
    pose.translate(spTranslation(x,y,0));
    Eigen::AngleAxisd rot(angle+SP_PI,Eigen::Vector3d::UnitZ());
    pose.rotate(rot);
    traj.AddWaypoint(pose);
  }

while(1){

  traj.UpdateCurves();
  gui_.Iterate(objects_);
}


}

void spirit::SenarioCeresTest() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.07));
  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
  car_param.pose.rotate(rot1);
//  Eigen::AngleAxisd rot2(M_PI/20,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot2);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.6;
  car_param.wheel_friction = 0.3;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

  spCtrlPts2ord_2dof inputcmd_curve;
  inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
  inputcmd_curve.col(1) = Eigen::Vector2d(0,10);
  inputcmd_curve.col(2) = Eigen::Vector2d(0,10);

  ceres::Problem problem;
  spStateVec targetstate;
  targetstate << 0.483252,   0.0525651 ,  0.0469795,     3.14148,     3.14151 ,   -1.70123 ,    9.85097 ,    2.05154,  -0.0211807 ,    0.34601 , -0.0193395 ,-0.00510512;
//  targetstate << 0.482476,    0.0519377,    0.0859993,  1.81986e-06,   2.0501e-06,     -1.44179,      9.81992,       2.0248, -4.98532e-06,     0.344223,  -1.4591e-06,  7.56714e-06;
//  targetstate << 0.482476,    0.0519377,    0.0469993,  1.81986e-06,   2.0501e-06,     -1.44179,      9.81992,       2.0248, -4.98532e-06,     0.344223,  -1.4591e-06,  7.56714e-06;
//  targetstate << 0.483252, 0.0525651 , -1.70123  , 9.85097  , 2.05154  , 0.34601;
//  targetstate << 1, 1,  0 ,    20 ,  30 , 1;

//  Eigen::VectorXd cost_surf(100);
//  for(int ii=0;ii<100;ii++) {
//    CarSimFunctor sims(car_param);
//    inputcmd_curve.col(2) = Eigen::Vector2d(-SP_PI_QUART+(SP_PI_HALF/100)*ii,10);
//    sims(0,10,0.1,inputcmd_curve,0,-1);
//    Eigen::VectorXd res = targetstate-sims.GetStateVec();
//    cost_surf[ii] = res.dot(res.transpose());

//  }
//  std::cout << "cost surf is \n" << cost_surf << std::endl;
//  SPERROREXIT("done");


   CarSimFunctor sims(car_param);
   sims(0,10,0.1,inputcmd_curve,0,-1);
   std::cout << "state is -> " << sims.GetStateVec().transpose() << std::endl;
  ceres::CostFunction* cost_function = new VehicleCeresCostFunc(car_param,targetstate);
//   ceres::NumericDiffOptions numericdiff_options;
//  numericdiff_options.relative_step_size = 1e-1;
//  ceres::CostFunction* cost_function =
//      new ceres::NumericDiffCostFunction<CarCostFunction, ceres::CENTRAL, 6, 6>(new CarCostFunction(car_param,targetstate),ceres::Ownership::TAKE_OWNERSHIP,6,numericdiff_options);
  double parameters[7];
  for (int ii = 0; ii < 6; ++ii) {
    parameters[ii] = inputcmd_curve.data()[ii];
  }
  double sim_length = 1.0;
  parameters[6] =  sim_length;
//  std::cout << "val is " << *parameters[0] << std::endl;
//    problem.AddResidualBlock(cost_function, NULL, inputcmd_curve.data());
  problem.AddResidualBlock(cost_function, NULL, parameters);
//  problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.9), parameters);

  std::vector<int> fix_param_vec;
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);
//  fix_param_vec.push_back(2);
//  fix_param_vec.push_back(3);
//  fix_param_vec.push_back(5);
  fix_param_vec.push_back(6);

  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(7,fix_param_vec);
//  problem.SetParameterization(inputcmd_curve.data(),subparam);
  problem.SetParameterization(parameters,subparam);

//  problem.SetParameterLowerBound(inputcmd_curve.data(),0,-SP_PI/4);
//  problem.SetParameterLowerBound(inputcmd_curve.data(),2,-SP_PI/4);
//  problem.SetParameterLowerBound(inputcmd_curve.data(),4,-SP_PI/4);
//  problem.SetParameterLowerBound(inputcmd_curve.data(),1,-100);
//  problem.SetParameterLowerBound(inputcmd_curve.data(),3,-100);
//  problem.SetParameterLowerBound(inputcmd_curve.data(),5,-100);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),0,SP_PI/4);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),2,SP_PI/4);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),4,SP_PI/4);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),1,100);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),3,100);
//  problem.SetParameterUpperBound(inputcmd_curve.data(),5,100);

//  problem.SetParameterLowerBound(parameters,0,-SP_PI/4);
//  problem.SetParameterLowerBound(parameters,2,-SP_PI/4);
//  problem.SetParameterLowerBound(parameters,4,-SP_PI/4);
//  problem.SetParameterLowerBound(parameters,1,-100);
//  problem.SetParameterLowerBound(parameters,3,-100);
//  problem.SetParameterLowerBound(parameters,5,-100);
//  problem.SetParameterUpperBound(parameters,0,SP_PI/4);
//  problem.SetParameterUpperBound(parameters,2,SP_PI/4);
//  problem.SetParameterUpperBound(parameters,4,SP_PI/4);
//  problem.SetParameterUpperBound(parameters,1,100);
//  problem.SetParameterUpperBound(parameters,3,100);
//  problem.SetParameterUpperBound(parameters,5,100);
//  problem.SetParameterUpperBound(parameters,6,5);
//  problem.SetParameterLowerBound(parameters,6,0.1);

  // Run the solver!
  ceres::Solver::Options options;
//  options.check_gradients = true;
//  options.gradient_check_numeric_derivative_relative_step_size = 0.05;
  options.update_state_every_iteration = true;
//  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (int ii = 0; ii < 6; ++ii) {
    inputcmd_curve.data()[ii] = parameters[ii];
  }
  sim_length = parameters[6];

  std::cout << summary.FullReport() << "\n";
  std::cout << "sol is \n" << inputcmd_curve << std::endl;
  CarSimFunctor sims2(car_param);
  sims2(0,(int)(10),0.1,inputcmd_curve,0,-1);
  std::cout << "state is -> " << sims2.GetStateVec().transpose() << std::endl;
  std::cout << "vs       -> " << targetstate.transpose() << std::endl;
  std::cout << "simulation length is " << sim_length << std::endl;
}


void spirit::CalcJacobianTest(spVehicleConstructionInfo& car_param,spPlannerJacobian& jacobian, spStateVec& end_state, const spCtrlPts2ord_2dof& cntrl_vars,unsigned int num_sim_steps,double sim_step_size, const spPose& init_pose, double fd_delta) {
  spObjectHandle obj_cars_ind = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(obj_cars_ind));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_cars_ind);
  car.SetSteeringServoTorque(1000);
  car.SetSteeringServoMaxVel(1000);
  car.SetEngineMaxVel(1000);
  car.SetEngineTorque(1000);

//  spCurve control_curve(3,2);
  spCurve control_curve(2,2);
  // 8+1 simulations required to fill the jacobian
  control_curve.SetBezierControlPoints(cntrl_vars);
  spPointXd sample_control(2);
  spPointXd curvature_cost(2);
  control_curve.Get2ndDrivativeCurveArea(curvature_cost);
  std::cout << "cur cost   " << curvature_cost.transpose() << std::endl;
//  car.SetPose(init_pose);
//  car.SetClampToSurfaceFlag();
//  physics_.Iterate(objects_,0.001);
//  spPose clamp_pose(car.GetPose());
//  spPose clamp_wheel[4];
//  for(int ii=0;ii<4;ii++){
//    car.GetWheel(ii)->SetRotVel(spRotVel(0,0,0));
//    car.GetWheel(ii)->SetLinVel(spLinVel(0,0,0));
//    car.GetWheel(ii)->SetAngle(0);
//    clamp_wheel[ii] = car.GetWheel(ii)->GetPose();
//  }
//  car.SetLinVel(spLinVel(0,0,0));
//  car.SetRotVel(spRotVel(0,0,0));
  // simulate final pose of vehicle with current control_vars
  for(int ii=1;ii<=num_sim_steps;ii++) {
    control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
    car.SetFrontSteeringAngle(sample_control[0]);
    car.SetEngineMaxVel(sample_control[1]);
    objects_.StepPhySimulation(sim_step_size);
    gui_.Iterate(objects_);
//    spGeneralTools::Delay_ms(100);
  }
  end_state = car.GetStateVecor();
  spWaypoint& waypoint1 = (spWaypoint&)(objects_.GetObject(obj_waypoint_index1));
//  std::cout << "pose is \n" << car.GetPose().matrix() << std::endl;
  waypoint1.SetPose(car.GetPose());
  gui_.RemoveObject(car);
  objects_.RemoveObj(obj_cars_ind);
  std::cout << "state vec is \n" << end_state.transpose() << std::endl;

  // now do the same thing with fd_delta applied to spCurve
  control_curve.SetBezierControlPoints(cntrl_vars);
  for(int jj=0;jj<4;jj++) {
    obj_cars_index[jj+1] = objects_.CreateVehicle(car_param);
//    gui_.AddObject(objects_.GetObject(obj_cars_index[jj+1]));
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_cars_index[jj+1]);
    car.SetSteeringServoTorque(1000);
    car.SetSteeringServoMaxVel(1000);
    car.SetEngineMaxVel(1000);
    car.SetEngineTorque(1000);

    // perturb a control signal
    double a = SP_PI;
    if(jj%2==1)
      a = 100;
    control_curve.PerturbControlPoint(jj+2,a*fd_delta);
    spPointXd sample_control(2);
    // simulate final pose of vehicle with perturbed control_vars
    for(int ii=1;ii<=num_sim_steps;ii++) {
      control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineMaxVel(sample_control[1]);
      objects_.StepPhySimulation(sim_step_size);
//      gui_.Iterate(objects_);
//      spGeneralTools::Delay_ms(100);
    }
//    gui_.RemoveObject(car);
    objects_.RemoveObj(obj_cars_index[jj+1]);

    spStateVec perturbed_state_delta = car.GetStateVecor();
    spPointXd pert_curvaturecost(2);
    control_curve.Get2ndDrivativeCurveArea(pert_curvaturecost);
    control_curve.RemoveLastPerturbation();
    Eigen::Vector2d diff = pert_curvaturecost-curvature_cost;
    // find forward finite difference value and put in jacobian
    jacobian.col(jj) << (perturbed_state_delta-end_state)*(1/(a*fd_delta)),diff;
  }
  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");;
  std::cout << "jacobian is : \n" << jacobian.format(OctaveFmt) << std::endl;
  Eigen::MatrixXd jtj(jacobian.transpose()*jacobian);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jtj);
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  std::cout << "jtj condition number is " << cond << std::endl;

}

void spirit::ScenarioPlannerTest() {
  spVehicleConstructionInfo car_param;

  // create and add a car
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.07));
  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
  car_param.pose.rotate(rot1);
//  Eigen::AngleAxisd rot2(M_PI/20,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot2);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.6;
  car_param.wheel_friction = 0.3;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

//  for (int ii = 0; ii < 9; ii++) {
//    obj_cars_index[ii] = objects_.CreateVehicle(car_param);
//    physics_.AddObject(objects_.GetObject(obj_cars_index[ii]));
//    gui_.AddObject(objects_.GetObject(obj_cars_index[ii]));
//    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_cars_index[ii]);
//    car.ClampToSurface();
//    physics_.Iterate(objects_,0.001);
//  }
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0, 0, -0.5));
//  Eigen::AngleAxisd ang(-M_PI / 10, Eigen::Vector3d::UnitX());
//  ground.rotate(ang);

  obj_gnd_index = objects_.CreateBox(ground, spBoxSize(10, 10, 1), 0, spColor(0, 1, 0));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
  spBox& gnd = (spBox&) objects_.GetObject(obj_gnd_index);
  gnd.SetFriction(1);
  gnd.SetPose(ground);
  spPose waypoint_pose(spPose::Identity());
  obj_waypoint_index1 =
      objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
  gui_.AddObject(objects_.GetObject(obj_waypoint_index1));
//  waypoint_pose.translate(spTranslation(1, 1, 0));
//  obj_waypoint_index2 =
//      objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index2));

  // test bezierplanner
//  spBezierPlanner bezplanner(&gui_);
//  bezplanner.AddWaypoint(waypoint1);
//  bezplanner.AddWaypoint(waypoint2);
//  bezplanner.IsLoop(false);
  spPlannerJacobian jacobian;
//  spCtrlPts3ord_2dof inputcmd_curve;
  spCtrlPts2ord_2dof inputcmd_curve;
  inputcmd_curve.col(0) = Eigen::Vector2d(0,10);
  inputcmd_curve.col(1) = Eigen::Vector2d(-SP_PI_QUART/10,5);
  inputcmd_curve.col(2) = Eigen::Vector2d(-SP_PI_QUART/10,30);
//  inputcmd_curve.col(3) = Eigen::Vector2d(-SP_PI_QUART/3,30);
double prev_norm = 100000;
for(int jj=0;jj<100;jj++) {
  std::cout << "jj is " << jj << std::endl;
  spPose Startpose(spPose::Identity());
  spStateVec state_vec;
  CalcJacobianTest(car_param,jacobian,state_vec,inputcmd_curve,10,0.1,Startpose,0.00001);
  spResidualVec z;
  z << 0.385925, 0.0905907 ,  -1.8649 ,  4.01045 ,  2.85222,  0.888181,-1.25664 ,20;
  Eigen::VectorXd vec_diag(8);
//  vec_diag << 0.1,0.1,0.1,0.1,0.1,0.1;
  vec_diag << 1,1,1,1,1,1,1,1;
  Eigen::MatrixXd R = vec_diag.asDiagonal();
  Eigen::MatrixXd jtj(jacobian.transpose()*R*jacobian);
  spCurve control_curve(2,2);
  // 8+1 simulations required to fill the jacobian
  control_curve.SetBezierControlPoints(inputcmd_curve);
  spResidualVec hx;
  spPointXd curvature_cost(2);
  control_curve.Get2ndDrivativeCurveArea(curvature_cost);
  hx << state_vec,curvature_cost;
  Eigen::VectorXd jtb(jacobian.transpose()*R*(Eigen::VectorXd)(z-hx));
  Eigen::VectorXd x_update = jtj.ldlt().solve(jtb);
  std::cout << "update is \n" << x_update.transpose() << std::endl;
  std::cout << "norm is " << x_update.norm() << std::endl;
  if(x_update.norm()>=prev_norm){
    std::cout << "done here" << std::endl;
    std::cout << "last pose is " << state_vec.transpose() << std::endl;
    break;
  }
  prev_norm = x_update.norm();
  std::cout << "solution is" << std::endl;
  for(int ii=0;ii<4;ii++) {
    inputcmd_curve.data()[ii+2] += 0.9*x_update[ii];
    std::cout << inputcmd_curve.data()[ii+2] << std::endl;
  }
}
while(1){
  gui_.Iterate(objects_);
}

}

double test_plant(double vin, spTimestamp curr_time) {
  static double cap_voltage = 0;
  static bool flag = false;
  static spTimestamp prev_time = spGeneralTools::Tick();
  if(flag == false){
    prev_time = curr_time;
    flag = true;
    return 0;
  }
  // limit vin
  if(vin>100) {
    vin = 100;
  }
  if(vin<-100) {
    vin = -100;
  }
  cap_voltage += (vin-cap_voltage)*(spGeneralTools::TickTock_ms(prev_time,curr_time)*0.001);
  prev_time = curr_time;
  return cap_voltage;
}

void PIDController_test() {
  // simulate rc circuit
//  for(int ii=0; ii<1000; ii++) {
//    double sim_result = test_plant(10,spGeneralTools::Tick());
//    std::cout << "sim output is " << sim_result << std::endl;
//    spGeneralTools::Delay_ms(10);
//  }
//  for(int ii=0; ii<1000; ii++) {
//    double sim_result = test_plant(5,spGeneralTools::Tick());
//    std::cout << "sim output is " << sim_result << std::endl;
//    spGeneralTools::Delay_ms(10);
//  }

  // test pid for rc circuit
  spPID rc_pid(10);
  double target_v = 10;
  double current_v = 0;
  rc_pid.SetGainP(20);
  rc_pid.SetGainI(12);
  rc_pid.SetGainD(0.1);
  for(int ii=0; ii<10000; ii++) {
    rc_pid.SetPlantError(target_v-current_v);
    double cntrl_effort = rc_pid.GetControlOutput();
    current_v = test_plant(cntrl_effort,spGeneralTools::Tick());
    std::cout << "sim output is " << current_v << std::endl;
    spGeneralTools::Delay_ms(10);
  }
}

void spirit::ScenarioPIDController() {
  spVehicleConstructionInfo car_param;

  //  PIDController_test();

  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.06));
  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
  car_param.pose.rotate(rot1);
//  Eigen::AngleAxisd rot2(M_PI/20,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot2);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.6;
  car_param.wheel_friction = 0.6;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

  obj_car_index = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(obj_car_index));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
  car.SetEngineMaxVel(10);
  car.SetEngineTorque(10);
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0, 0, -0.5));
  obj_gnd_index = objects_.CreateBox(ground, spBoxSize(10, 10, 1), 0, spColor(0, 1, 0));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
  spBox& gnd = (spBox&) objects_.GetObject(obj_gnd_index);
  gnd.SetFriction(1);
  spPose waypoint_pose(spPose::Identity());
//  obj_waypoint_index0 = objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index0));
//  waypoint_pose.translate(spTranslation(1, 1, 0));
//  obj_waypoint_index1 = objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index1));
//  spWaypoint& waypoint0 = (spWaypoint&) objects_.GetObject(obj_waypoint_index0);
//  spWaypoint& waypoint1 = (spWaypoint&) objects_.GetObject(obj_waypoint_index1);
//  // example hermite between two waypoints
//  spCtrlPts3ord_3dof pts;
//  pts.col(0) = waypoint0.GetPose().translation();
//  pts.col(1) = waypoint0.GetPose().rotation()*spTranslation(1,0,0)*waypoint0.GetLength();
//  pts.col(2) = waypoint1.GetPose().translation();
//  pts.col(3) = waypoint1.GetPose().rotation()*spTranslation(1,0,0)*waypoint1.GetLength();
//  spCurve curve(3,3);
//  curve.SetHermiteControlPoints(pts);
//  spPoints3d line_pts(50);
//  curve.GetPoints3d(line_pts);
//  obj_linestrip_index = objects_.CreateLineStrip(waypoint0.GetPose(), line_pts, spColor(1, 0, 0));
//  gui_.AddObject(objects_.GetObject(obj_linestrip_index));
//  spLineStrip& strip = (spLineStrip&) objects_.GetObject(obj_linestrip_index);

  spTrajectory traj(gui_,objects_);
  spPose pose(spPose::Identity());
  traj.AddWaypoint(pose);
  pose.translate(spTranslation(1,1,0));
  traj.AddWaypoint(pose);
  pose.translate(spTranslation(1,-1,0));
  traj.AddWaypoint(pose);
  pose.translate(spTranslation(-1,-1,0));
  traj.AddWaypoint(pose);

while(1){
//  pts.col(0) = waypoint0.GetPose().translation();
//  pts.col(1) = waypoint0.GetPose().rotation()*spTranslation(1,0,0)*waypoint0.GetLength();
//  pts.col(2) = waypoint0.GetPose().translation();
//  pts.col(3) = waypoint0.GetPose().rotation()*spTranslation(1,0,0)*waypoint0.GetLength();
//  curve.SetHermiteControlPoints(pts);
//  curve.GetPoints3d(line_pts);
//  strip.SetLineStripPoints(line_pts);

  traj.UpdateCurves();
  gui_.Iterate(objects_);
//  objects_.StepPhySimulation(0.001);
}

}

void spirit::ScenarioWorldCarFall() {
  // create and add a car
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.06));
//  Eigen::AngleAxisd rot1(M_PI/4+0.17355,Eigen::Vector3d::UnitX());
//  car_param.pose.rotate(rot1);
//  Eigen::AngleAxisd rot2(SP_PI/2,Eigen::Vector3d::UnitZ());
//  car_param.pose.rotate(rot2);
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.6;
  car_param.wheel_friction = 0.6;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 2;
  car_param.steering_servo_upper_limit = SP_PI / 2;

  for (int ii = 0; ii < 1; ii++) {
    obj_car_index = objects_.CreateVehicle(car_param);
    gui_.AddObject(objects_.GetObject(obj_car_index));
    spAWSDCar& car = (spAWSDCar&)objects_.GetObject(obj_car_index);
//    car.SetClampToSurfaceFlag();
    car.SetEngineTorque(10);
    car.SetEngineMaxVel(100);
    car.SetSteeringServoMaxVel(1);
    car.SetSteeringServoTorque(100);
    car.SetFrontSteeringAngle(SP_PI/2);
//    car.SetFrontSteeringAngle(0);
//    car.SetRearSteeringAngle(0);
//    car_param.pose.translate(spTranslation(0.1,0,0));
  }
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0, 0, -0.5));
//  Eigen::AngleAxisd ang(-M_PI / 10, Eigen::Vector3d::UnitX());
//  ground.rotate(ang);

  obj_gnd_index = objects_.CreateBox(ground, spBoxSize(10, 10, 1), 0, spColor(0, 1, 0));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
  spBox& gnd = (spBox&)objects_.GetObject(obj_gnd_index);
  gnd.SetFriction(1);
//  spPose waypoint_pose(spPose::Identity());
//  obj_waypoint_index1 =
//      objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index1));

//  waypoint_pose.translate(spTranslation(1, 1, 0));
//  obj_waypoint_index2 =
//      objects_.CreateWaypoint(waypoint_pose, spColor(0, 0, 1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index2));

  // example bezier between 4 waypoints
//  spPose bezpos(spPose::Identity());
//  spCtrlPts3ord_3dof pts;
//  pts.col(0) = spPoint3d(0, 0, 0);
//  pts.col(1) = spPoint3d(1, 2, 1);
//  pts.col(2) = spPoint3d(2, -2, -1);
//  pts.col(3) = spPoint3d(3, 0, 2);
//  spCurve curve(3,3);
//  curve.SetBezierControlPoints(pts);
//  spPoints3d line_points;
//  curve.GetPoints3d(line_points,10);
//  obj_linestrip_index = objects_.CreateLineStrip(bezpos, line_points, spColor(1, 0, 0));
//  gui_.AddObject(objects_.GetObject(obj_linestrip_index));

//  // example hermite between two waypoints
//  spPose bezpos(spPose::Identity());
//  spWaypoint& waypoint1 = (spWaypoint&) objects_.GetObject(obj_waypoint_index1);
//  spWaypoint& waypoint2 = (spWaypoint&) objects_.GetObject(obj_waypoint_index2);
//  spCtrlPts3ord_3dof pts;
//  pts.col(0) = waypoint1.GetPose().translation();
//  pts.col(1) = waypoint1.GetPose().rotation()*spTranslation(1,0,0)*waypoint1.GetLength();
//  pts.col(2) = waypoint2.GetPose().translation();
//  pts.col(3) = waypoint2.GetPose().rotation()*spTranslation(1,0,0)*waypoint2.GetLength();
//  spCurve curve(3,3);
//  curve.SetHermiteControlPoints(pts);
//  spPoints3d line_points;
//  curve.GetPoints3d(line_points,50);
//  obj_linestrip_index = objects_.CreateLineStrip(bezpos, line_points, spColor(1, 0, 0));
//  gui_.AddObject(objects_.GetObject(obj_linestrip_index));
}

/*
 // this function has been revised to test the multi threaded jacobian
calculation
void spirit::ScenarioWorldCarFall() {
  // create and add a car
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spVehicleConfig::AWSD;
  car_param.pose.translate(spTranslation(0,0,0.24));
//  Eigen::AngleAxisd rot(M_PI/4+0.17355,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot);
  car_param.wheels_anchor.push_back(spTranslation(-0.13,0.17,-0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13,-0.17,-0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13,-0.17,-0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13,0.17,-0.003));
  car_param.chassis_size = spBoxSize(0.2,0.42,0.05);
  car_param.cog = spTranslation(0,0,0);
  car_param.wheel_friction = 100;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 10;
  car_param.susp_stiffness = 100;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 3;
  car_param.steering_servo_lower_limit = -SP_PI/2;;
  car_param.steering_servo_upper_limit = SP_PI/2;;
  for(int ii=0;ii<42;ii++) {
    Physics phy;
    phy.Create(user_settings_.GetPhysicsEngineType());
    Objects phy_obj;
    int car = phy_obj.CreateVehicle(car_param);
    phy.AddObject(phy_obj.GetObject(car));
    spPose ground(spPose::Identity());
    ground.translate(spTranslation(0,0,-0.5));
    phy.AddObject(phy_obj.GetObject(phy_obj.CreateBox(ground,spBoxSize(10,10,1),0,spColor(0,1,0))));
    physics_vec_.push_back(phy);
    objects_vec_.push_back(phy_obj);
  }
  std::cout << "size of parallell world is: " << physics_vec_.size() <<
std::endl;
}
*/
void spirit::ScenarioWorldBoxFall() {

  spPose pose(spPose::Identity());
  pose.translate(spTranslation(0, 0, 2));
//  Eigen::AngleAxisd ang(M_PI / 5, Eigen::Vector3d::UnitY());
//  pose.rotate(ang);

  obj_box_index = objects_.CreateBox(pose, spBoxSize(1, 1, 1), 1, spColor(1, 0, 0));
  gui_.AddObject(objects_.GetObject(obj_box_index));

  //  std::cout << "box gui index is " << objects_.GetObject(obj_box_index).GetGuiIndex() << std::endl;
//  physics_.AddObject(objects_.GetObject(obj_box_index));
//  spBox& box = (spBox&)objects_.GetObject(obj_box_index);
//  box.ClampToSurface();

  // create and add a ground as a box to objects_ vector
  obj_gnd_index = objects_.CreateBox(spPose::Identity(), spBoxSize(10, 10, 1),0, spColor(0, 1, 0));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
}

//spCtrlPts3ord_3dof ptss;
//spCurve curve(3,3);

//void spirit::LocalPlanTest(spWaypoint& w1, spWaypoint& w2) {
//  // define a bezier curve for control_commands and initialize it with some
//  // curve ctrl points but don't need to add to gui since we dont wat to show
//  // this.
//  spPose bezpos(spPose::Identity());
//  spBezierCtrlPoints2D pts;
//  pts.col(0) = spPoint(0, 0);
//  pts.col(1) = spPoint(0, 1);
//  pts.col(2) = spPoint(0, 2);
//  pts.col(3) = spPoint(0, 3);

//  int control_indx =
//      objects_.CreateBezierCurve(bezpos, pts, spColor(0, 1, 0));
//  spCubicBezierCtrlPoints<2> pts;
//    pts.col(0) = Eigen::Vector2d(0, 0);
//    pts.col(1) = Eigen::Vector2d(0, 1);
//    pts.col(2) = Eigen::Vector2d(0, 2);
//    pts.col(3) = Eigen::Vector2d(0, 3);

//  spAWSDCar& car = (spAWSDCar&)objects_.GetObject(obj_car_index);

//}

void spirit::IterateWorld() {
  //  spTimestamp gui_tick = spGeneralTools::Tick();
  gui_.Iterate(objects_);
  //  double gui_cost = spGeneralTools::Tock_ms(gui_tick);
  //  std::cout << "Gui Iteration time:   " << gui_cost << "ms" << std::endl;
  static int fl = 0;

  // local planner tests
//  spBezierPlanner plan;
//  spWaypoint planpoint;
//  spPose pose(spPose::Identity());
//  for(int ii=0;ii<3;ii++) {
//    planpoint.SetPose(pose);
//    plan.AddWaypoint(planpoint);
//    pose.translate(spTranslation(0,1,0));
//  }
//  planpoint.SetPose(pose);
//  plan.AddWaypoint(planpoint,2);
//  planpoint = plan.GetWaypoint(20);
//  std::cout << "waypoint is " << planpoint.GetPose().translation() <<
//               std::endl;
//  plan.UpdateCurves();
  spGeneralTools::Delay_ms(100);
SPERROREXIT("ENDOFPROGRAM");
  if (fl<500) {
//    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
//    car.GetStateVecor();
//    spPose pose(spPose::Identity());
//    pose.translate(spTranslation(1, 1, 1));

//    if(fl==300)
//      car.SetPose(pose);
//      car.GetWheel(3)->SetAngle(3*SP_PI/4);
//    std::cout << "wheel is \n" << car.GetWheel(1)->GetPose().rotation() << std::endl;
    spTimestamp phy_tick = spGeneralTools::Tick();
//    spBox& box1 = (spBox&) objects_.GetObject(obj_gnd_index);
//    std::cout << "z is " << box1.GetPose().translation()[2] << std::endl;
//    objects_.StepPhySimulation(0.01);

    //    std::cout << "wheel pose is\n" << car.GetWheel(0)->GetPose().matrix() << std::endl;
    fl++;
    double phy_cost = spGeneralTools::Tock_us(phy_tick);
//    std::cout << "Phy Iteration time:   " << phy_cost/1000 << "ms" << std::endl;

//    Eigen::MatrixXd A(2,4);
//    A = Eigen::MatrixXd::Random(2,4);
//    std::cout << "a is: \n" << A << std::endl;
//    Eigen::VectorXd b(Eigen::Map<Eigen::VectorXd>(A.data(), A.cols()*A.rows()));
//    std::cout << "b is \n" << b << std::endl;
//    A.data()[7] = 2;
//    std::cout << "a 2 is now:\n" << A.data()[2] << std::endl;

//    // sstate vector test
//    Eigen::Array<double,1,13> a(car.GetStateVecor());
//    std::cout << std::fixed;
//    std::cout << std::setprecision(2);
//    std::cout << "state is:\n" << a << std::endl;
//    spGeneralTools::Delay_ms(2000);
  }


  /*
    // test multithreaded jacobian calculation
    // spirit private function
    void spirit::runthread(int ind) {
      physics_vec_[ind].Iterate(objects_vec_[ind]);
    }
    // spirit.h defenitions
    std::vector<Physics> physics_vec_;
    std::vector<Objects> objects_vec_;
    void runthread(int ind);
    std::thread threads_[42];

    // iteration of jacobian calculation
    for(int jj=0;jj<5;jj++) {
      for(int ii=0;ii<42;ii++) {
        threads_[ii] = std::thread(&spirit::runthread,this,ii);
      }
      for(int ii=0;ii<42;ii++) {
        threads_[ii].join();
      }
    }
  */

  // example hermite between two waypoints
//  spWaypoint& waypoint1 = (spWaypoint&) objects_.GetObject(obj_waypoint_index1);
//  spWaypoint& waypoint2 = (spWaypoint&) objects_.GetObject(obj_waypoint_index2);
//  ptss.col(0) = waypoint1.GetPose().translation();
//  ptss.col(1) = waypoint1.GetPose().rotation()*spTranslation(1,0,0)*waypoint1.GetLength();
//  ptss.col(2) = waypoint2.GetPose().translation();
//  ptss.col(3) = waypoint2.GetPose().rotation()*spTranslation(1,0,0)*waypoint2.GetLength();
//  curve.SetHermiteControlPoints(ptss);
//  spLineStrip& line = (spLineStrip&) objects_.GetObject(obj_linestrip_index);
//  spPoints3d line_points;
//  curve.GetPoints3d(line_points,50);
//  line.SetLineStripPoints(line_points);

//    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
  //  if(fl>100) {
  //    car.SetLocalCOG(spTranslation(0,-0.3,0));
//    std::cout << "pose is \n" << waypoint.GetPose().matrix() << std::endl;
  //  }

}
