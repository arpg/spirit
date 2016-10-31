#include <spirit/Objects/spLineStrip.h>

spLineStrip::spLineStrip() {
  mass_ = 0;
  color_ = spColor(1, 1, 1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::LINESTRIP;
}

spLineStrip::~spLineStrip() {}

void spLineStrip::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_guichanged_ = true;
}

const spPose& spLineStrip::GetPose() { return pose_; }

void spLineStrip::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spLineStrip::GetColor() {
  return color_;
}

void spLineStrip::SetLineStripPoints(const spPoints3d& pts/*in their local coordinates*/) {
  points_ = pts;
  obj_guichanged_ = true;
}

const spPoints3d& spLineStrip::GetLineStripPoints() {
  return points_;
}
