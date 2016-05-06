#include <spirit/Settings.h>

using namespace spirit;

Settings::Settings()
  : gui_type_(GUI_NONE)
{}

Settings::~Settings() {}

void Settings::SetGuiType(const spGuiType& gui_type) {
  gui_type_ = gui_type;
}

bool Settings::HasGui() {
  return (gui_type_ != GUI_NONE);
}

spGuiType Settings::GetGuiType() {
  return gui_type_;
}
