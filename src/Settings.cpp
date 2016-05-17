#include <spirit/Settings.h>

using namespace spirit;

Settings::Settings()
  : gui_type_(GUI_NONE)
{}

Settings::~Settings() {}

void Settings::SetGuiType(const spGuiType& gui_type) {
  gui_type_ = gui_type;
}

spGuiType Settings::GetGuiType() {
  return gui_type_;
}

spPhyEngineType Settings::GetPhysicsEngineType( void ) {
  return phy_type_;
}

void Settings::SetPhysicsEngineType(const spPhyEngineType &phy_type) {
  phy_type_ = phy_type;
}

