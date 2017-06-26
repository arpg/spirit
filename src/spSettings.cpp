#include <spirit/spSettings.h>

spSettings::spSettings()
  : gui_type_(spGuiType::GUI_NONE)
{
  num_threads_ = 1;
}

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

void spSettings::SetNumThreads(unsigned int num_threads) {
  num_threads_ = num_threads;
}

int spSettings::GetNumThreads() {
  return num_threads_;
}
