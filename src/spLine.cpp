#include <spirit/Objects/spLine.h>

spLine::spLine() {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::LINE;
}

spLine::~spLine() {}

void spLine::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spLine::GetPose(){
  return pose_;
}

void spLine::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spLine::GetColor() {
  return color_;
}
