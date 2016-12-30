#include <spirit/spirit.h>
#include <iomanip>
spirit::spirit(spSettings& user_settings) { user_settings_ = user_settings; }

spirit::~spirit() {}

void spirit::Create() {
  // create gui object if requested
  if (user_settings_.GetGuiType() != spGuiType::GUI_NONE) {
    gui_.Create(user_settings_.GetGuiType());
  }
  // add physics world
  if (user_settings_.GetPhysicsEngineType() != spPhyEngineType::PHY_NONE) {
    physics_.Create(user_settings_.GetPhysicsEngineType());
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
spVehicleConstructionInfo car_param;

double f1(double x){
  return 100*(10-(x))*(10-(x));
}
double f2(double x){
  return 0.1*(2-(x))*(2-(x));
}

double fd1(double x, double step){
  return (f1(x+step)-f1(x))/step;
}
double fd2(double x, double step){
  return (f2(x+step)-f2(x))/step;
}

void spirit::ScenarioGNTest() {
  Eigen::VectorXd x_curr(2);
  x_curr.setOnes();
  x_curr*=0.1;
for(int ii=0;ii<100;ii++) {
  Eigen::MatrixXd jac(2,2);
  double del = 0.000001;
  jac(0,0) = fd1(x_curr[0],del*x_curr[0]);
  jac(0,1) = 0;//fd1(x_curr[1],0.001);
  jac(1,0) = 0;//fd2(x_curr[0],0.001);
  jac(1,1) = fd2(x_curr[1],del*x_curr[1]);

  Eigen::MatrixXd jtj(2,2);
  jtj = jac.transpose()*jac;
  Eigen::VectorXd b(2);
  b(0) = -f1(x_curr[0]);
  b(1) = -f2(x_curr[1]);
  Eigen::VectorXd jtb(jac.transpose()*b);
  Eigen::VectorXd x_update(jtj.ldlt().solve(jtb));
  x_curr += x_update;
  std::cout << "x is -> \n" << x_curr << "\n ii is: " << ii<< std::endl;
  if(x_update.norm()<0.00001)
    break;
}
}


void spirit::CalcJacobianTest(spPlannerJacobian& jacobian, spStateVec& end_state, const spCtrlPts3ord_2dof& cntrl_vars,unsigned int num_sim_steps,double sim_step_size, const spPose& init_pose, double fd_delta) {
  obj_cars_index[0] = objects_.CreateVehicle(car_param);
  physics_.AddObject(objects_.GetObject(obj_cars_index[0]));
  gui_.AddObject(objects_.GetObject(obj_cars_index[0]));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_cars_index[0]);

  car.SetEngineTorque(1000);
  car.SetSteeringServoTorque(100);
  car.SetSteeringServoMaxVel(100);

  spCurve control_curve(3,2);
  // 8+1 simulations required to fill the jacobian
  control_curve.SetBezierControlPoints(cntrl_vars);
  spPointXd sample_control(2);
//  car.SetPose(init_pose);
  car.SetClampToSurfaceFlag();
//  physics_.Iterate(objects_,0.001);
  spPose clamp_pose(car.GetPose());
  spPose clamp_wheel[4];
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
    physics_.Iterate(objects_,sim_step_size);
    gui_.Iterate(objects_);
//    spGeneralTools::Delay_ms(100);
  }
  end_state = car.GetStateVecor();

  spWaypoint& waypoint1 = (spWaypoint&) objects_.GetObject(obj_waypoint_index1);
  waypoint1.SetPose(car.GetPose());

  // now do the same thing with fd_delta applied to spCurve
  control_curve.SetBezierControlPoints(cntrl_vars);

  for(int jj=0;jj<8;jj++) {
    obj_cars_index[jj+1] = objects_.CreateVehicle(car_param);
    physics_.AddObject(objects_.GetObject(obj_cars_index[jj+1]));
    gui_.AddObject(objects_.GetObject(obj_cars_index[jj+1]));
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_cars_index[jj+1]);
    car.SetEngineTorque(1000);
    car.SetSteeringServoTorque(100);
    car.SetSteeringServoMaxVel(100);

//    car.SetPose(spPos?:Le(spPose::Identity()));
    car.SetClampToSurfaceFlag();
//    physics_.Iterate(objects_,0.001);
//    car.SetPose(clamp_pose);
//    for(int ii=0;ii<4;ii++){
//      car.GetWheel(ii)->SetPose(clamp_wheel[ii]);
//      car.GetWheel(ii)->SetRotVel(spRotVel(0,0,0));
//      car.GetWheel(ii)->SetLinVel(spLinVel(0,0,0));
//      car.GetWheel(ii)->SetAngle(0);
//    }
//    car.SetLinVel(spLinVel(0,0,0));
//    car.SetRotVel(spRotVel(0,0,0));


    // set state of vehicle
//    car.SetPose(init_pose);
//    car.ClampToSurface();
    int ii=0;
    while(ii--) {
        gui_.Iterate(objects_);
    }
    // perturb a control signal
    double a = SP_PI;
    if(jj%2==1)
      a = 1000;
    control_curve.PerturbControlPoint(jj,a*fd_delta);
    spPointXd sample_control(2);
    // simulate final pose of vehicle with perturbed control_vars
    for(int ii=1;ii<=num_sim_steps;ii++) {
      control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineMaxVel(sample_control[1]);
      physics_.Iterate(objects_,sim_step_size);
      gui_.Iterate(objects_);
//      spGeneralTools::Delay_ms(100);
    }
    control_curve.RemoveLastPerturbation();
    spStateVec perturbed_state_delta = car.GetStateVecor();
    // find forward finite difference value and put in jacobian
    jacobian.col(jj) = (perturbed_state_delta-end_state)/(a*fd_delta);
  }
  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");;
  std::cout << "jacobian is : \n" << jacobian.format(OctaveFmt) << std::endl;
  Eigen::MatrixXd jtj(jacobian.transpose()*jacobian);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jtj);
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  std::cout << "jtj condition number is " << cond << std::endl;
}

void spirit::ScenarioPlannerTest() {
  // create and add a car
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0));
//  Eigen::AngleAxisd rot1(M_PI/4+0.17355,Eigen::Vector3d::UnitX());
//  car_param.pose.rotate(rot1);
//  Eigen::AngleAxisd rot2(M_PI/20,Eigen::Vector3d::UnitY());
//  car_param.pose.rotate(rot2);
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
//  car_param.susp_upper_limit = 0.013;
//  car_param.susp_lower_limit = -0.028;
  car_param.susp_upper_limit = 0;
  car_param.susp_lower_limit = 0;
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
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));

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
  spCtrlPts3ord_2dof inputcmd_curve;
//  inputcmd_curve.col(0) = Eigen::Vector2d(-SP_PI_HALF,0);
//  inputcmd_curve.col(1) = Eigen::Vector2d(-0.25*SP_PI_HALF,10);
//  inputcmd_curve.col(2) = Eigen::Vector2d(0.25*SP_PI_HALF,100);
//  inputcmd_curve.col(3) = Eigen::Vector2d(SP_PI_HALF,200);
  inputcmd_curve.col(0) = Eigen::Vector2d(0,100);
  inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
  inputcmd_curve.col(2) = Eigen::Vector2d(0,0);
  inputcmd_curve.col(3) = Eigen::Vector2d(0,100);
for(int jj=0;jj<10;jj++) {
  spPose Startpose(spPose::Identity());
  spStateVec hx;
  CalcJacobianTest(jacobian,hx,inputcmd_curve,10,0.1,Startpose,0.0001);
  std::cout << "hx is " << hx << std::endl;
  spStateVec z;
  z << 0,2,0.0600042,0.999405,0,0,0.0344877,-0.857567,19.596,0.000207682,0,0,1.61495;
//  z <<  0.246979,1.2,0.0600042,0.999405,7.42208e-06,-1.11024e-06,0.0344877,-0.857567,19.596,0.000207682,4.495e-05,2.07909e-06,1.61495;
//  z << 0.100207816,1.45602,0.0599957,1,-1.61806e-06,-2.98278e-07,0.000543913,-0.00760412,30.6918,-0.0229283,-0.00734955,-0.00855209,0.00156885;
  Eigen::MatrixXd R(13,13);
  R = Eigen::MatrixXd::Identity(13,13);
//  R.row(3).setZero();
//  R.row(4).setZero();
//  R.row(5).setZero();
//  R.row(6).setZero();
//  R.row(7).setZero();
//  R.row(8).setZero();
//  R.row(9).setZero();
//  R.row(10).setZero();
//  R.row(11).setZero();
//  R.row(12).setZero();
  Eigen::MatrixXd jtj(jacobian.transpose()*R*jacobian);
  Eigen::VectorXd jtb(jacobian.transpose()*R*(Eigen::VectorXd)(z-hx));
  Eigen::VectorXd x_update = jtj.ldlt().solve(jtb);
  std::cout << "update is " << x_update << std::endl;
  for(int ii=0;ii<8;ii++) {
    inputcmd_curve.data()[ii] += x_update[ii];
  }
}

}

void spirit::ScenarioWorldCarFall() {
  // create and add a car
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0));
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
  car_param.wheel_rollingfriction = 0;
  car_param.wheel_friction = 0.3;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
//  car_param.susp_upper_limit = 0.13;
//  car_param.susp_lower_limit = -0.28;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 2;
  car_param.steering_servo_upper_limit = SP_PI / 2;

  for (int ii = 0; ii < 1; ii++) {
    obj_car_index = objects_.CreateVehicle(car_param);
    physics_.AddObject(objects_.GetObject(obj_car_index));
    gui_.AddObject(objects_.GetObject(obj_car_index));
    spAWSDCar& car = (spAWSDCar&)objects_.GetObject(obj_car_index);
    car.SetClampToSurfaceFlag();
    car.SetEngineTorque(10);
    car.SetEngineMaxVel(0);
    car.SetSteeringServoMaxVel(1);
    car.SetSteeringServoTorque(100);
    car.SetFrontSteeringAngle(SP_PI/4);
//    car.SetFrontSteeringAngle(0);
    car.SetRearSteeringAngle(0);
    car_param.pose.translate(spTranslation(0.1,0,0));
  }
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0, 0, -0.5));
//  Eigen::AngleAxisd ang(-M_PI / 10, Eigen::Vector3d::UnitX());
//  ground.rotate(ang);

  obj_gnd_index = objects_.CreateBox(ground, spBoxSize(10, 10, 1), 0, spColor(0, 1, 0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));

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
  pose.translate(spTranslation(0, 0, 6));
//  Eigen::AngleAxisd ang(M_PI / 5, Eigen::Vector3d::UnitY());
//  pose.rotate(ang);
  obj_box_index = objects_.CreateBox(pose, spBoxSize(1, 1, 1), 1, spColor(1, 0, 0));
  physics_.AddObject(objects_.GetObject(obj_box_index));
  spBox& box = (spBox&)objects_.GetObject(obj_box_index);
  box.ClampToSurface();
  gui_.AddObject(objects_.GetObject(obj_box_index));

  // create and add a ground as a box to objects_ vector
  obj_gnd_index = objects_.CreateBox(spPose::Identity(), spBoxSize(10, 10, 1),
                                     0, spColor(0, 1, 0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
}

spCtrlPts3ord_3dof ptss;
spCurve curve(3,3);

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
  spGeneralTools::Delay_ms(50);

  if (fl<300) {
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
    if(fl==0)
      car.GetWheel(3)->SetAngle(3*SP_PI/4);
//    std::cout << "wheel is \n" << car.GetWheel(1)->GetPose().rotation() << std::endl;
    spTimestamp phy_tick = spGeneralTools::Tick();
    physics_.Iterate(objects_,0.01);
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
