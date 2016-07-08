#include <spirit/spirit.h>
#include <chrono>
#include <thread>

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
  spPose pose(spPose::Identity());
  pose.translate(spTranslation(0,0,1));
//  Eigen::AngleAxisd rot(M_PI/10,Eigen::Vector3d::UnitY());
//  pose.rotate(rot);
  // set wheel anchors
  std::vector<spTranslation> anchors;
  anchors.push_back(spTranslation(-0.13,0.17,-0.18));
  anchors.push_back(spTranslation(-0.13,-0.17,-0.18));
  anchors.push_back(spTranslation(0.13,-0.17,-0.18));
  anchors.push_back(spTranslation(0.13,0.17,-0.18));
  obj_car_index = objects_.CreateVehicle(pose,anchors,spColor(0,1,0));
  physics_.AddObject(objects_.GetObject(obj_car_index));
  gui_.AddObject(objects_.GetObject(obj_car_index));


  // create and add a ground as a box to objects_ vector
  spPose ground(spPose::Identity());
  ground.translate(spTranslation(0,0,-0.5));
  Eigen::AngleAxisd ang(M_PI/20,Eigen::Vector3d::UnitX());
//  ground.rotate(ang);

  obj_gnd_index = objects_.CreateBox(ground,spBoxSize(10,10,1),0,spColor(0,1,0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));

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

void spirit::IterateWorld() {
  gui_.Iterate(objects_);
  static int fl = 0;
  if(fl<10000) {
    physics_.Iterate(objects_);
    fl++;
  }
  spVehicle& car = (spVehicle&) objects_.GetObject(obj_car_index);
  if(fl>100) {
//    car.GetWheel(0)->SetSteeringServoTargetAngle(SP_PI/10);
  }
//  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
