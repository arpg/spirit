#include <spirit/Objects/spBox.h>

spBox::spBox() {
  mass_ = 0;
  rolling_friction = 0;
  friction = 1;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  dims_ = spBoxSize(1,1,1);
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  object_type_ = spObjectType::BOX;
}

spBox::~spBox() {}

void spBox::SetDimensions(const spBoxSize& dims) {
  dims_ = dims;
  obj_guichanged_ = true;
  obj_phychanged_ = true;
}

spBoxSize spBox::GetDimensions() {
  return dims_;
}


void spBox::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spBox::GetPose(){
  return pose_;
}

void spBox::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spBox::GetColor() {
  return color_;
}


void spBox::SetMass(double mass) {
  mass_ = mass;
  obj_phychanged_ = true;
}

double spBox::GetMass() {
  return mass_;
}

void spBox::ClampToSurface() {
  obj_clamptosurface_ = true;
}
