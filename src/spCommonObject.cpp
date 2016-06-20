#include <spirit/Objects/spCommonObject.h>

spCommonObject::spCommonObject() {}

spCommonObject::~spCommonObject() {}

void spCommonObject::SetPhyIndex(int index) {
  if (index < 0) {
    std::cerr << "Line:" << __LINE__ << " File: " << __FILE__
              << " Error: Wrong Index for Phy Object Detected" << std::endl;
  }

  index_phy_ = index;
}

void spCommonObject::SetGuiIndex(int index) { index_gui_ = index; }

int spCommonObject::GetGuiIndex() { return index_gui_; }

int spCommonObject::GetPhyIndex() { return index_phy_; }

bool spCommonObject::HasChangedGui() {
  bool status = obj_guichanged_;
  obj_guichanged_ = false;
  return status;
}

// tells if object has been modified since last query
bool spCommonObject::HasChangedPhy() {
  bool status = obj_phychanged_;
  obj_phychanged_ = false;
  return status;
}

bool spCommonObject::IsCompound()
{
  return is_compound;
}

spObjectType spCommonObject::GetObjecType() { return object_type_; }
