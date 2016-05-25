#include <spirit/spirit.h>
#include <chrono>
#include <thread>

spirit::spirit(spSettings& user_settings): boxpose_(spPose::Identity()) {
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

void spirit::ScenarioWorldBoxFall() {
  // create and add a ground as a box to objects_ vector
  obj_gnd_index = objects_.CreateBox(spPose::Identity(),spBoxSize(20,20,0.1),0,spColor(0, 1, 0));
  physics_.AddObject(objects_.GetObject(obj_gnd_index));
  gui_.AddObject(objects_.GetObject(obj_gnd_index));

  boxpose_.translate(spVector3d(0,0,1));
  obj_box_index = objects_.CreateBox(boxpose_,spBoxSize(1,1,1),1,spColor(0, 1, 0));
  physics_.AddObject(objects_.GetObject(obj_box_index));
  gui_.AddObject(objects_.GetObject(obj_box_index));
}

void spirit::IterateWorld() {
  boxpose_.translate(spVector3d(0,0,-0.01));
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  objects_.GetObject(obj_box_index).SetPose(boxpose_);
  gui_.Iterate(objects_);
  physics_.Iterate();
}
