#include <spirit/World.h>

using namespace spirit;

World::World(Settings& user_settings){
  user_settings_ = user_settings;
}

World::~World(){

}

void World::Create() {
  // create gui object if requested
  if(user_settings_.GetGuiType() != spGuiType::GUI_NONE){
    gui_.Create(user_settings_.GetGuiType());
  }
  // add physics world
  if(user_settings_.GetPhysicsEngineType() != spPhyEngineType::PHY_NONE){
    physics_.Create(user_settings_.GetPhysicsEngineType());
  }
}

bool World::ShouldRun() {
  if(gui_.ShouldQuit()) {
    return false;
  } else {
    return true;
  }
}

void World::IterateGraphics() {
  gui_.Refresh();
  gui_.CheckKeyboardAction();
}

void World::CheckKeyboardAction() {
  gui_.CheckKeyboardAction();
}

void World::TestWorldBoxFall() {
  spBox box;
  box.dims << 1,1,1;
  box.mass = 1;
  box.pose.translate(spVector3d(0,0,10));
  box.color = spVector3d(1,0,0);
  physics_.AddBox(box);
}

void World::IterateWorld() {
  IterateGraphics();
  IteratePhysics();
}

void World::IteratePhysics(){
}
