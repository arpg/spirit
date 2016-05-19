#include <spirit/Objects/spBox.h>

spBox::spBox() {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
}

spBox::~spBox() {}

void spBox::SetDimensions(const spBoxSize& dims) {
  dims_ = dims;
}

spBoxSize spBox::GetDimensions() {
  return dims_;
}


void spBox::SetPose(const spPose& pose) {
  pose_ = pose;
}

const spPose& spBox::GetPose(){
  return pose_;
}

void spBox::SetColor(const spColor& color) {
  color_ = color;
}

void spBox::SetMass(double mass) {
  mass_ = mass;
}

double spBox::GetMass() {
  return mass_;
}
