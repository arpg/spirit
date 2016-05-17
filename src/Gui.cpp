#include <spirit/Gui.h>

Gui::Gui(){

}

Gui::~Gui(){

}

void Gui::Create(const spGuiType gui_type) {
  switch(gui_type) {
    case spGuiType::GUI_NONE:
      std::cout << "Initializing spirit withoud gui" << std::endl;
      break;
    case spGuiType::GUI_PANGOSCENEGRAPH:
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
