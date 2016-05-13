#include <spirit/World.h>

using namespace spirit;

World::World(Settings& user_settings){
  user_settings_ = std::make_shared<Settings>(user_settings);
}

World::~World(){

}

void World::Create() {
  // create gui object if requested
  if(user_settings_->HasGui()){
    gui_.reset();
    gui_ = std::make_shared<Gui>();
    gui_->Create(user_settings_->GetGuiType());
  }
  // add physics world

}

bool World::ShouldRun() {
  if(gui_->ShouldQuit()) {
    return false;
  } else {
    return true;
  }
}

void World::IterateGraphics() {
  gui_->Refresh();
  gui_->CheckKeyboardAction();
}

void World::CheckKeyboardAction() {
  gui_->CheckKeyboardAction();
}
