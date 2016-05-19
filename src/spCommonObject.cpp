#include <spirit/Objects/spCommonObject.h>

// initialize static variable spirit_obj_counter
int spCommonObject::spirit_obj_counter_ = 0;

spCommonObject::spCommonObject() {
  spirit_obj_index_ = spirit_obj_counter_++;
}

spCommonObject::~spCommonObject() {
  spirit_obj_counter_--;
}

int spCommonObject::GetSpiritObjIndex() {
  return spirit_obj_index_;
}

void spCommonObject::SetPhysicsObjIndex(int index) {
  phy_obj_index_ = index;
}

void spCommonObject::SetGraphicsObjIndex(int index) {
  graphics_obj_index_ = index;
}
