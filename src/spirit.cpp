#include <spirit/spirit.h>

spirit::spirit(const spSettings& user_settings) {
  user_settings_ = user_settings;
  // create gui object if requested
  if (user_settings_.GetGuiType() != spGuiType::GUI_NONE) {
    gui_.Create(user_settings_.GetGuiType());
  }
  // add physics world
  if (user_settings_.GetPhysicsEngineType() != spPhyEngineType::PHY_NONE) {
//    physics_.Create(user_settings_.GetPhysicsEngineType());
  }

  // Initialize default car parameters
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose = spPose::Identity();
  car_param.pose.translate(spTranslation(0,0,0.07));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.1;
  car_param.wheel_friction = 0.6;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.05;//0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.engine_torque = 100;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;
  car_param.steering_servo_max_velocity = 100;
  car_param.steering_servo_torque = 100;
}

spirit::~spirit() {}

bool spirit::ShouldRun() {
  if (gui_.ShouldQuit()) {
    return false;
  } else {
    return true;
  }
}

void spirit::CheckKeyboardAction() { gui_.CheckKeyboardAction(); }

void spirit::NonlinControl() {
  // Create vehicle
  spObjectHandle car_handle = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);
  spPose pose = spPose::Identity();
  pose.translation() = spTranslation(-6,0,0.06);
  car.SetPose(pose);
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
  car.SetChassisLinearVelocity(spLinVel(0,3,0));
  while(1) {
    // Measure current states
    double k_v = 10;
    double k_d = 50;
    double k_p = 50;
    double v_d = 2;
    double r_d = 3;
    double r = 2;
    double x = car.GetPose().translation()[0];
    double y = car.GetPose().translation()[1];
    double etha = SP_PI-atan2(y,x);
    Eigen::Matrix3d rotmat = car.GetPose().rotation();
    double cai = -atan2(rotmat(1,0),rotmat(0,0));
    double v = car.GetChassisLinearVelocity().norm();
    double v_perp = v*sin(cai-etha);
    double v_tan = v*cos(cai-etha);
    double a_perp;
    double a_tan;
//    std::cout << "etha -> " << etha << "\t\tcai-> " << cai << std::endl;

    // Calculate controls
    a_perp = (v_tan*v_tan)/r - k_d*v_perp-k_p*(r_d-r);
    a_tan = -k_v*(v_tan-v_d);


    double u1 = a_tan*cos(etha-cai) - a_perp*sin(etha-cai);
    double u2 = a_tan*sin(etha-cai)/(v*v) + a_perp*cos(etha-cai)/(v*v);

    if(u2 < -SP_PI_QUART) u2 = -SP_PI_QUART;
    if(u2 > SP_PI_QUART) u2 = SP_PI_QUART;
//    if(u1 < -0.01) u1 = -0.01;
//    if(u1 > 0.01) u1 = 0.01;
    if(u1 < -30) u1 = -30;
    if(u1 > 30) u1 = 30;
    std::cout << "u1 -> " << u1 << "\t\tu2 -> " << u2 << std::endl;
    car.SetFrontSteeringAngle(u2);
    car.SetEngineMaxVel(u1);
//    car.SetEngineTorque(u1);
//    car.SetFrontSteeringAngle(SP_PI_QUART/2);
//    car.SetEngineMaxVel(3);

    objects_.StepPhySimulation(0.1);
    gui_.Iterate(objects_);
  }
}

void spirit::DummyTests() {
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
  gui_.AddObject(objects_.GetObject(gnd_handle));

  car_param.steering_servo_max_velocity = 100;
  car_param.steering_servo_torque = 100;
  for(int ii=0; ii<1000; ii++) {
    spObjectHandle car_handle = objects_.CreateVehicle(car_param);
    gui_.AddObject(objects_.GetObject(car_handle));
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);
    car.SetEngineMaxVel(10);
    car.SetRearSteeringAngle(0);
    car.SetFrontSteeringAngle(SP_PI/4);
  }

//  spGeneralTools::Delay_ms(10);
  while(1){
    spTimestamp tt =  spGeneralTools::Tick();
    objects_.StepPhySimulation(0.1);
    double time =  spGeneralTools::Tock_us(tt);
    std::cout << "time is " << time  << std::endl;
    spState state;
//    state = car.GetState();
//    car.SetState(state);
//    gui_.Iterate(objects_);
  };
}


void spirit::SenarioCostSurf() {

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));

  spCtrlPts2ord_2dof inputcmd_curve;
  inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
  inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
  inputcmd_curve.col(2) = Eigen::Vector2d(0,0);

  spState startstate;
  startstate.pose.translate(spTranslation(2,0,0.07));
  spState goalstate;
  goalstate.pose.translate(spTranslation(0,2,0.07));
  Eigen::AngleAxisd rot1(M_PI/2,Eigen::Vector3d::UnitZ());
  goalstate.pose.rotate(rot1);
  goalstate.linvel = spLinVel(-100,0,0);

  CarSimFunctor sims(car_param,startstate);
  sims(0,20,0.1,inputcmd_curve,0,-1);
  spState endstate = sims.GetState();
  std::ofstream myfile;
  myfile.open("example.csv");
  for(int ii=0; ii<200; ii+=5) {
    for(int jj=-78; jj<79; jj+=5){
//    int jj=0;
      inputcmd_curve.col(2) = Eigen::Vector2d(jj/100.0,ii);
      sims(0,10,0.1,inputcmd_curve,0,-1);
      spState errorstate = goalstate - sims.GetState();
      double a = goalstate.linvel.dot(sims.GetState().linvel);
      a /= goalstate.linvel.norm()*sims.GetState().linvel.norm();
      double r = 0;
//      for(int kk=0;kk<6;kk++) {
//        r += std::pow(errorstate.vector()[kk],2);
//      }
//      r += std::pow((1-a),2);
//      r += std::pow(0.02*(goalstate.linvel.norm()-sims.GetState().linvel.norm()),2);
//      std::cout << "norm diff is " << goalstate.linvel.norm()-sims.GetState().linvel.norm() << std::endl;
      for(int kk=6;kk<9;kk++) {
        r += std::pow(0.02*errorstate.vector()[kk],2);
      }
      myfile << r << ",";
    }
    myfile << "\n";
  }
  myfile.close();
}

void spirit::ScenarioSteadyStateCircle(){
  spTrajectory traj(gui_,objects_);
  spInputInstance2D init_input;
  init_input << 0, 10;
  spCirclePlanner planner(car_param,1.5,init_input,1,10,&gui_);
  planner.SolveInitialPlan(traj);
  while(1){
    gui_.Iterate(objects_);
  }
}

void spirit::SenarioTrajectoryTest() {

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));

  spTrajectory traj(gui_,objects_);
  // put waypoints on a elliptical path
  double a = 4;
  double b = 2;
  int num_waypoints = 8;
  for(int ii=0; ii<num_waypoints; ii++) {
    // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
    double theta = ii*(2*SP_PI)/num_waypoints;
    double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
    double x = r*cos(theta);
    double y = r*sin(theta);
    // slope of the line is
    double angle = atan2(-(x*b*b),(y*a*a));
    spPose pose(spPose::Identity());
    pose.translate(spTranslation(x,y,0.07));
    Eigen::AngleAxisd rot(angle+SP_PI_HALF,Eigen::Vector3d::UnitZ());
    pose.rotate(rot);
//    if(ii==1){
//      Eigen::AngleAxisd rot2(SP_PI_QUART,Eigen::Vector3d::UnitY());
//      pose.rotate(rot2);
//    }
    traj.AddWaypoint(pose,20);
  }
  gui_.Iterate(objects_);
  traj.IsLoop(true);
//  std::cout << "waypoint speed is \n" << traj.GetWaypoint(0).GetLinearVelocity() << std::endl;

  spLocalPlanner localplanner(car_param,false,&gui_);

  for(int ii=0; ii<traj.GetNumWaypoints(); ii++) {
    if(ii != 0) {
      // set the solution from last control point from ii'th trajectory to first control point of ii+1'th trajectory
      // this will guarantee C1 continuity
//      traj.GetControls(ii).data()[0] = traj.GetControls(ii-1).data()[4];
//      traj.GetControls(ii).data()[1] = traj.GetControls(ii-1).data()[5];
    }
    localplanner.SolveInitialPlan(traj,ii);
    gui_.Iterate(objects_);
    localplanner.SolveLocalPlan(traj,ii);
//    if(ii==1)
//    for(int jj=0;jj<50;jj++) {
//      traj.PlaybackTrajectoryOnGUI(car_param,0,3);
//      traj.PlaybackTrajectoryOnGUI(car_param,1,3);
//    }
    gui_.Iterate(objects_);
//    spObjectHandle thewayobj = objects_.CreateWaypoint(state.pose,spColor(0,1,0));
//    ((spWaypoint&)objects_.GetObject(thewayobj)).SetLinearVelocityNorm(state.linvel.norm());
//    gui_.AddObject(objects_.GetObject(thewayobj));
  }
int i=0;
  while(1){
    traj.PlaybackTrajectoryOnGUI(car_param,i,1);
    if(i<traj.GetNumWaypoints()-1)
      i++;
    else
      i=0;
    gui_.Iterate(objects_);
  }

}

void spirit::zibil() {
  car_param.wheel_friction = 0.0;
//  car_param.chassis_mass = 180;
  car_param.steering_servo_max_velocity = 100;
  car_param.steering_servo_torque = 10;
  car_param.engine_torque = 0.01;
//    car_param.pose.translate(spTranslation(0,0,1));
  spObjectHandle car1_handle = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(car1_handle));
  spAWSDCar& car1 = (spAWSDCar&) objects_.GetObject(car1_handle);
  car1.SetEngineMaxVel(0);
  car1.SetRearSteeringAngle(0);
  car1.SetFrontSteeringAngle(0);

//  car_param.pose.translate(spTranslation(0.2,0,0));
//  car_param.engine_torque = 0.01;
//  spObjectHandle car2_handle = objects_.CreateVehicle(car_param);
//  gui_.AddObject(objects_.GetObject(car2_handle));
//  spAWSDCar& car2 = (spAWSDCar&) objects_.GetObject(car2_handle);
//  car2.SetEngineMaxVel(40);
//  car2.SetRearSteeringAngle(0);
//  car2.SetFrontSteeringAngle(SP_PI_QUART);

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
//  objects_.StepPhySimulation(0.01);
  spState st(car1.GetState());
  st.linvel = spLinVel(0,10,0);
  car1.SetState(st);
  car1.SetEngineMaxVel(0);
  objects_.StepPhySimulation(0.1);

int cnt = 0;
  while(1){
    if(cnt == 300) {
//      car1.SetState(car2.GetState());
      cnt = 0;
    } else {
      cnt++;
    }
//    objects_.StepPhySimulation(0.01);
    gui_.Iterate(objects_);
  }
}

void spirit::SenarioCalibrationTest() {

}

void spirit::SenarioControllerTest() {
  // Create vehicle
  spObjectHandle car_handle = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);

  spTrajectory traj(gui_,objects_);
  // put waypoints on a elliptical path
  double a = 1;
  double b = 1;
  int num_waypoints = 8;
  for(int ii=0; ii<num_waypoints; ii++) {
    // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
    double theta = ii*(2*SP_PI)/num_waypoints;
    double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
    double x = r*cos(theta);
    double y = r*sin(theta);
    // slope of the line is
    double angle = atan2(-(x*b*b),(y*a*a));
    spPose pose(spPose::Identity());
    pose.translate(spTranslation(x,y,0.07));
    Eigen::AngleAxisd rot(angle+SP_PI_HALF+0.8,Eigen::Vector3d::UnitZ());
    pose.rotate(rot);
    traj.AddWaypoint(pose,3);
    spRotVel rotvel(0,0,4);
    traj.GetWaypoint(ii).SetRotVel(rotvel);
    traj.GetWaypoint(ii).SetLinearVelocityDirection(spLinVel(0.4,1,0));
  }
  gui_.Iterate(objects_);
  traj.IsLoop(true);

  spLocalPlanner localplanner(car_param,false,&gui_);
  spBVPWeightVec weight_vec;
  weight_vec << 10, 10, 10, 0.1, 0.1, 1, 0.09, 0.09, 0.09, 0.1, 0.1, 0.1,0.1;
  localplanner.SetCostWeight(weight_vec);
  for(int ii=0; ii<traj.GetNumWaypoints(); ii++) {
    traj.SetTravelDuration(ii,0.3);
    localplanner.SolveInitialPlan(traj,ii);
//    localplanner.SolveLocalPlan(traj,ii);
    gui_.Iterate(objects_);
  }

  // set driving car's first pose
  spPose car_init_pose(traj.GetWaypoint(0).GetPose());
  car_init_pose.translate(spTranslation(0,0,0));
  car.SetPose(car_init_pose);

//  while(1){
//    gui_.Iterate(objects_);
//  }
  spCtrlPts2ord_2dof controls;
  controls.col(0) = Eigen::Vector2d(0,0);
  controls.col(1) = Eigen::Vector2d(0,10);
  controls.col(2) = Eigen::Vector2d(0,20);

  // create a MPC controller with horizon
  float horizon = 0.5;
  spMPC mpc(car_param,horizon);

  while(1){
    spTimestamp t0 = spGeneralTools::Tick();
    if(mpc.CalculateControls(traj,car.GetState(),controls)) {
      spCurve controls_curve(2,2);
      spPointXd next_control(2);
      controls_curve.SetBezierControlPoints(controls);
      // only get the control signal for next point and apply to car
      controls_curve.GetPoint(next_control,DISCRETIZATION_STEP_SIZE/horizon);
      std::cout << "next signals are " << std::fixed << std::setprecision(3) << next_control.transpose() << std::endl;
      car.SetFrontSteeringAngle(next_control[0]);
      car.SetEngineMaxVel(next_control[1]);
      controls.col(0) = next_control;
      double calc_time = spGeneralTools::Tock_ms(t0);
      std::cout << "calc time was " << calc_time << std::endl;

      objects_.StepPhySimulation(DISCRETIZATION_STEP_SIZE);
      gui_.Iterate(objects_);
    } else {
      SPERROREXIT("No Controls could be calculated! ");
    }
  }
}



void spirit::multithreadtest() {
  spState initstate;
  initstate.pose.translate(spTranslation(0,0,0.06));
  std::vector<std::shared_ptr<CarSimFunctor>> sims;
  std::vector<std::shared_ptr<spStateSeries>> sim_traj;
  spCtrlPts2ord_2dof cntrl_vars;
  double simulation_length = 0.5;
  for (int ii = 0; ii < 4; ii++) {
    sims.push_back(std::make_shared<CarSimFunctor>(car_param,initstate));
  }
  std::cout << "all carsimfunctor objects created" << std::endl;
  int ii = 0;
//  CarSimFunctor sim0(car_param,initstate);
//  CarSimFunctor sim1(car_param,initstate);

while(1) {
  std::cout << "while start " << ii++ << std::endl;
//  sim0.RunInThread(0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
//  sim1.RunInThread(0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
//  sim0.WaitForThreadJoin();
//  sim1.WaitForThreadJoin();

  sims[0]->RunInThread(0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
  sims[1]->RunInThread(1,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
  sims[2]->RunInThread(2,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
  sims[3]->RunInThread(3,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0);
  sims[0]->WaitForThreadJoin();
  sims[1]->WaitForThreadJoin();
  sims[2]->WaitForThreadJoin();
  sims[3]->WaitForThreadJoin();

#if 0
  for (int ii = 0; ii < 2; ii++) {
    std::cout << "pushing back object " << ii << std::endl;
    sims[ii]->SetState(initstate);
    sim_traj.push_back(std::make_shared<spStateSeries>());
    thread_vec.push_back(std::make_shared<std::thread>(std::ref(*(sims[ii].get())),ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, std::ref(cntrl_vars), FINITE_DIFF_EPSILON, ii, sim_traj[ii]));
  }
//  std::cout << "pushing last thread" << std::endl;
//  sims[6]->SetState(initstate);
//  sim_traj.push_back(std::make_shared<spStateSeries>());
//  thread_vec.push_back(std::make_shared<std::thread>(std::ref(*sims[6].get()),6,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, std::ref(cntrl_vars), FINITE_DIFF_EPSILON, -1, sim_traj[sim_traj.size()-1]));

  std::cout << "done pushing all threads" << std::endl;
  for (int ii = 0; ii <= 1; ii++) {
      std::cout << "joining thread " << ii << std::endl;
      thread_vec[ii]->join();
  }
#endif
  std::cout << "done in thread----------------------------------------------" << std::endl;
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
  spPID rc_pid;
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

  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
  car_param.pose.rotate(rot1);

  obj_car_index = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(obj_car_index));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
  car.SetEngineMaxVel(10);
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0, 0, -0.5));
  obj_gnd_index = objects_.CreateBox(ground, spBoxSize(10, 10, 1), 0, spColor(0, 1, 0));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
  spBox& gnd = (spBox&) objects_.GetObject(obj_gnd_index);
  gnd.SetFriction(1);
//  spPose waypoint_pose(spPose::Identity());
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
  traj.AddWaypoint(pose,true);
  pose.translate(spTranslation(1,1,0));
  traj.AddWaypoint(pose,true);
  pose.translate(spTranslation(1,-1,0));
  traj.AddWaypoint(pose,true);
  pose.translate(spTranslation(-1,-1,0));
  traj.AddWaypoint(pose,true);

while(1){
//  pts.col(0) = waypoint0.GetPose().translation();
//  pts.col(1) = waypoint0.GetPose().rotation()*spTranslation(1,0,0)*waypoint0.GetLength();
//  pts.col(2) = waypoint0.GetPose().translation();
//  pts.col(3) = waypoint0.GetPose().rotation()*spTranslation(1,0,0)*waypoint0.GetLength();
//  curve.SetHermiteControlPoints(pts);
//  curve.GetPoints3d(line_pts);
//  strip.SetLineStripPoints(line_pts);

//  traj.UpdateCurves();
  gui_.Iterate(objects_);
//  objects_.StepPhySimulation(0.001);
}

}

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
//    spTimestamp phy_tick = spGeneralTools::Tick();
//    spBox& box1 = (spBox&) objects_.GetObject(obj_gnd_index);
//    std::cout << "z is " << box1.GetPose().translation()[2] << std::endl;
//    objects_.StepPhySimulation(0.01);

    //    std::cout << "wheel pose is\n" << car.GetWheel(0)->GetPose().matrix() << std::endl;
    fl++;
//    double phy_cost = spGeneralTools::Tock_us(phy_tick);
//    std::cout << "Phy Iteration time:   " << phy_cost/1000 << "ms" << std::endl;

  }

}
