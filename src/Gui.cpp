#include <spirit/Gui.h>

Gui::Gui(){

}

Gui::~Gui(){

}

void Gui::Create(const spirit::spGuiType gui_type) {
  switch(gui_type) {
    case spirit::GUI_PANGOSCENEGRAPH:
      gui_.reset();
      gui_ = std::make_shared<spPangolinScenegraphGui>();
      break;
    default:
      std::cerr << "Wrong type of GUI has been selected" << std::endl;
  }
  gui_->InitGui();
}

bool Gui::ShouldQuit() {
  return gui_->ShouldQuit();
}

void Gui::Refresh() {
  gui_->Refresh();
}
void Gui::CheckKeyboardAction() {
  gui_->CheckKeyboardAction();
}
