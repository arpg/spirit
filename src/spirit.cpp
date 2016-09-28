#include <spirit/spirit.h>
#include <spirit/Objects/spBezierCurve.h>

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

void spirit::CheckKeyboardAction() {
  gui_.CheckKeyboardAction();
}

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

  for(int ii=0;ii<1;ii++) {
    obj_car_index = objects_.CreateVehicle(car_param);
    physics_.AddObject(objects_.GetObject(obj_car_index));
    gui_.AddObject(objects_.GetObject(obj_car_index));
//    car_param.pose.translate(spTranslation(0.1,0,0));
  }
  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0,0,-0.5));
  Eigen::AngleAxisd ang(M_PI/20,Eigen::Vector3d::UnitX());
//  ground.rotate(ang);

  obj_gnd_index = objects_.CreateBox(ground,spBoxSize(10,10,1),0,spColor(0,1,0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));

//  spPose waypoint_pose(spPose::Identity());
//  obj_waypoint_index = objects_.CreateWaypoint(waypoint_pose,spColor(0,0,1));
//  gui_.AddObject(objects_.GetObject(obj_waypoint_index));
  spPose bezpos(spPose::Identity());
  spBezierCtrlPoints pts;
  pts.col(0) = spPoint(0,0,0);
  pts.col(1) = spPoint(1,2,1);
  pts.col(2) = spPoint(2,-2,-1);
  pts.col(3) = spPoint(3,0,2);
  obj_curve_index = objects_.CreateBezierCurve(bezpos,pts,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(obj_curve_index));
//  spBezierCurve& bez = (spBezierCurve&) objects_.GetObject(obj_curve_index);


}

void spirit::ScenarioWorldBoxFall() {
  spPose pose(spPose::Identity());
  pose.translate(spTranslation(0,0,8));
  Eigen::AngleAxisd ang(M_PI/5,Eigen::Vector3d::UnitY());
  pose.rotate(ang);
  obj_box_index = objects_.CreateBox(pose,spBoxSize(1,1,1),1,spColor(1,0,0));
  physics_.AddObject(objects_.GetObject(obj_box_index));
  gui_.AddObject(objects_.GetObject(obj_box_index));

  // create and add a ground as a box to objects_ vector
  obj_gnd_index = objects_.CreateBox(spPose::Identity(),spBoxSize(10,10,1),0,spColor(0,1,0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));
}

spBezierCtrlPoints ptss;

void spirit::IterateWorld() {
  spTimestamp gui_tick = spGeneralTools::Tick();
  gui_.Iterate(objects_);
  double gui_cost = spGeneralTools::Tock_ms(gui_tick);
  std::cout << "Gui Iteration time:   " << gui_cost << "ms" << std::endl;
  static int fl = 0;

  if(fl<1) {
    spTimestamp phy_tick = spGeneralTools::Tick();
    physics_.Iterate(objects_);
    double phy_cost = spGeneralTools::Tock_ms(phy_tick);
    std::cout << "Phy Iteration time:   " << phy_cost << "ms" << std::endl;
    fl++;
    spPose ps(spPose::Identity());
    spBezierCtrlPoints pts;
    pts.col(0) = spPoint(0,0,0);
    pts.col(1) = spPoint(1,2,0);
    pts.col(2) = spPoint(2,-2,0);
    pts.col(3) = spPoint(3,0,0);
    spBezierCurve curve;
    curve.SetControlPoints(pts);
    curve.SetPose(ps);
    spPoints p;
    curve.GetPoints(p,10);
//    std::cout << "num points -> " << p.size() << std::endl;
    spPoint point = p[2];
//    std::cout << "data \n" << point << std::endl;
  }

  spBezierCurve& bez = (spBezierCurve&) objects_.GetObject(obj_curve_index);
  ptss.col(0) = spPoint(0,0,0);
  ptss.col(1) = spPoint(1,2,1);
  ptss.col(2) = spPoint(2,-2,-1);
  double rand = ((std::rand()%100)/100.0);
  ptss.col(3) = spPoint(3,0,2+rand);
  bez.SetControlPoints(ptss);
//  spPose bezpos(bez.GetPose());
//  bezpos.translate(spTranslation(0.001,0,0));
//  bez.SetPose(bezpos);

//  spWaypoint& waypoint = (spWaypoint&) objects_.GetObject(obj_waypoint_index);
//  spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_car_index);
//  if(fl>100) {
//    car.SetLocalCOG(spTranslation(0,-0.3,0));
//  std::cout << "pose is \n" << waypoint.GetPose().matrix() << std::endl;
//  }
  spGeneralTools::Delay_ms(10);
}
