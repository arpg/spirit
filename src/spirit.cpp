#include <spirit/spirit.h>

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

void spirit::IterateGraphics() {
  gui_.Refresh();
  gui_.CheckKeyboardAction();
}

void spirit::CheckKeyboardAction() {
  gui_.CheckKeyboardAction();
}

void spirit::ScenarioWorldBoxFall() {
  // add a box for ground
  groundbox.SetDimensions(spBoxSize(1,1,2));
//  groundbox.dims << 50, 3, 50;
  // make it static
  groundbox.SetMass(0);
  groundbox.SetPose(spPose::Identity());
  groundbox.SetColor(spColor(0, 1, 0));
  spPose gnd_pose = spPose::Identity();
  groundbox.SetPose(gnd_pose);
  physics_.AddBox(groundbox);
  gui_.AddBox(groundbox);

  spBoxSize box_size;
  box_size << 1,1,1;
  box.SetDimensions(box_size);
  box.SetMass(1);
  spPose box_pose = spPose::Identity();
  box_pose.translate(spVector3d(0,0,10));
  box.SetPose(box_pose);
  box.SetColor(spColor(1, 0, 0));
  physics_.AddBox(box);
  gui_.AddBox(box);
}

void spirit::IterateWorld() {
  IterateGraphics();
    IteratePhysics();
}

void spirit::IteratePhysics() {}
