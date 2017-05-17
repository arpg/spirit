#include <spirit/spirit.h>
#include <spirit/CarSimFunctor.h>
//#include <iomanip>
#include <spirit/Planners/spTrajectory.h>
#include <spirit/VehicleCeresCostFunc.h>
#include <engine.h>
#include <fstream>

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

void spirit::DummyTests() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  spPose car_pose(spPose::Identity());
  car_pose.translate(spTranslation(0,0,0.06));
//  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
//  car_pose.rotate(rot1);
  car_param.pose = car_pose;

  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.1;
  car_param.wheel_friction = 0.8;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 10;
  car_param.susp_stiffness = 100;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.4;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
  gui_.AddObject(objects_.GetObject(gnd_handle));

  spObjectHandle car_handle = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);
  car.SetEngineTorque(10);
  car.SetEngineMaxVel(0);
  car.SetSteeringServoMaxVel(100);
  car.SetSteeringServoTorque(100);
  car.SetRearSteeringAngle(0);
  car.SetFrontSteeringAngle(SP_PI/4);


//  spGeneralTools::Delay_ms(10);
  while(1){
    objects_.StepPhySimulation(0.1);
    spState state;
    state = *car.GetState();
//    car.SetState(state);
    gui_.Iterate(objects_);
  };
}

void spirit::SenarioStateInitialization() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  spPose car_pose(spPose::Identity());
  car_pose.translate(spTranslation(0,0,0.06));
//  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
//  car_pose.rotate(rot1);
  car_param.pose = car_pose;

  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.1;
  car_param.wheel_friction = 0.4;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.057;
  car_param.susp_damping = 10;
  car_param.susp_stiffness = 100;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.4;
  car_param.chassis_mass = 5;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;

  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
  gui_.AddObject(objects_.GetObject(gnd_handle));

  spObjectHandle car_handle = objects_.CreateVehicle(car_param);
  gui_.AddObject(objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);
  car.SetEngineTorque(10);
  car.SetEngineMaxVel(40);
  car.SetSteeringServoMaxVel(100);
  car.SetSteeringServoTorque(100);
  car.SetRearSteeringAngle(0);
  car.SetFrontSteeringAngle(SP_PI/4);
//gui_.RemoveObject(objects_.GetObject(car_handle));
  spObjectHandle car2_handle;
  for(int ii=0;ii<10000;ii++) {
    if(ii%300 == 0) {
      if(ii != 0) {
        gui_.RemoveObject(objects_.GetObject(car2_handle));
        objects_.RemoveObj(car2_handle);
      }
      car2_handle = objects_.CreateVehicle(car_param);
      gui_.AddObject(objects_.GetObject(car2_handle));
      spAWSDCar& car2 = (spAWSDCar&) objects_.GetObject(car2_handle);
      car2.SetEngineTorque(10);
      car2.SetEngineMaxVel(40);
      car2.SetSteeringServoMaxVel(100);
      car2.SetSteeringServoTorque(100);
      car2.SetRearSteeringAngle(0);
      car2.SetFrontSteeringAngle(SP_PI/4);
//      state.pose.translate(spTranslation(0,0,0.01));
//      spState state = car.GetState();
      car2.SetState(*car.GetState());
    }
    objects_.StepPhySimulation(0.01);
    gui_.Iterate(objects_);
    spGeneralTools::Delay_ms(10);
  }
  while(1){
    gui_.Iterate(objects_);
  };
}

void spirit::SenarioCostSurf() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.3;
  car_param.wheel_friction = 0.4;
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
//  std::string PlotCommand = "x=[0 1 2 3 4 5];y=[0 1 4 9 16 25];plot(x, y);";
//  void * vpDcom = NULL;
//  int iReturnValue;
//  Engine* matlabEngine;
//  if (!(matlabEngine = engOpen("\0"))) {
//        fprintf(stderr, "\nCan't start MATLAB engine\n");
//  }
//  engEvalString(matlabEngine, PlotCommand.c_str());

//  Engine m_pEngine = engOpen(NULL);
//  double a[10];
//  mxArray *A;
//  A=mxCreateDoubleMatrix(1, 10, mxREAL);
//  memcpy((void *)mxGetPr(A), (void *)a, sizeof(double)*10);
//  engPutVariable(m_pEngine, "a", A);

//  spGeneralTools::PlotXY(value);

//  spTrajectory traj(gui_,objects_);
//  spPose pose = spPose::Identity();
//  pose.translate(spTranslation(0,0,0.05));
//  traj.AddWaypoint(pose,false);
//  Eigen::AngleAxisd rot1(-M_PI/4,Eigen::Vector3d::UnitZ());
//  pose.rotate(rot1);
//  pose.translate(spTranslation(1,1,0));
//  traj.AddWaypoint(pose,false);

//  spLocalPlanner localplanner(traj,car_param);
//  localplanner.CalcInitialPlans();

//  spCtrlPts2ord_2dof inputcmd_curve;
//  inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
//  inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
//  inputcmd_curve.col(2) = Eigen::Vector2d(0,0);
//  spState state;
//  state.pose = pose;
//  CarSimFunctor sim(car_param,state);
//  inputcmd_curve.col(2) = Eigen::Vector2d(0,0);
//  sim(0,10,0.1,inputcmd_curve,0,-1);
//  spState endstate = sim.GetState();
//  for(int ii=-100;ii<101;ii++) {
//    inputcmd_curve.col(2) = Eigen::Vector2d(0,0);
//    sim(0,10,0.1,inputcmd_curve,0,-1);
//    spState endstate = sim.GetState();

//  }


}


void spirit::SenarioTrajectoryTest() {
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  car_param.cog = spTranslation(0, 0, 0);
  car_param.chassis_friction = 0;
  car_param.wheel_rollingfriction = 0.3;
  car_param.wheel_friction = 0.8;
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

  spLocalPlanner localplanner(car_param,&gui_);


//  spPose pose(spPose::Identity());
//  Eigen::AngleAxisd rot1(-M_PI/2,Eigen::Vector3d::UnitZ());
//  pose.rotate(rot1);
//  std::cout << "rot is \n" << pose.rotation() << std::endl;
//  Eigen::AngleAxisd angleaxis(pose.rotation());
//  std::cout << "angle axis is \n" << angleaxis.matrix() << std::endl;
//  std::cout << "angle is " << angleaxis.angle() << std::endl;
//  std::cout << "axis is " << angleaxis.axis() << std::endl;

  for(int ii=0; ii<5/*traj.GetNumWaypoints()*/; ii++) {
    if(ii != 0) {
      // set the solution from last control point from ii'th trajectory to first control point of ii+1'th trajectory
      // this will guarantee C1 continuity
//      traj.GetControls(ii).data()[0] = traj.GetControls(ii-1).data()[4];
//      traj.GetControls(ii).data()[1] = traj.GetControls(ii-1).data()[5];
    }
    localplanner.SolveInitialPlan(traj,ii);
    gui_.Iterate(objects_);
    localplanner.SolveLocalPlan(traj,ii,true);
    if(ii==1)
    for(int jj=0;jj<50;jj++) {
      traj.PlaybackTrajectoryOnGUI(car_param,ii,0.4);
    }
    gui_.Iterate(objects_);
//    spObjectHandle thewayobj = objects_.CreateWaypoint(state.pose,spColor(0,1,0));
//    ((spWaypoint&)objects_.GetObject(thewayobj)).SetLinearVelocityNorm(state.linvel.norm());
//    gui_.AddObject(objects_.GetObject(thewayobj));
  }

  while(1){
//    traj.UpdateCurves();
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

void spirit::ScenarioWorldCarFall() {
  // create and add a car
  spVehicleConstructionInfo car_param;
  car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  car_param.pose.translate(spTranslation(0, 0, 0.06));
  Eigen::AngleAxisd rot1(M_PI/4+0.17355,Eigen::Vector3d::UnitX());
  car_param.pose.rotate(rot1);
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
    spPose carpose(spPose::Identity());
    carpose.translate(spTranslation(0,0,1));
    car.SetPose(carpose);
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
