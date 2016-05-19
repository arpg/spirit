#include <spirit/spSettings.h>

spSettings::spSettings()
  : gui_type_(GUI_NONE)
{}

spSettings::~spSettings() {}

void spSettings::SetGuiType(const spGuiType& gui_type) {
  gui_type_ = gui_type;
}

spGuiType spSettings::GetGuiType() {
  return gui_type_;
}

spPhyEngineType spSettings::GetPhysicsEngineType( void ) {
  return phy_type_;
}

void spSettings::SetPhysicsEngineType(const spPhyEngineType &phy_type) {
  phy_type_ = phy_type;
}

