#include <spirit/Objects/spWaypoint.h>

spWaypoint::spWaypoint(const spPose& pose, const spColor& color) {
  mass_ = 0;
  color_ = color;
  pose_ = pose;
  length_ = 1;
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = true;
  object_type_ = spObjectType::WAYPOINT;
}

spWaypoint::spWaypoint(const spWaypoint& wp) {
  length_ = wp.length_;
  pose_ = wp.pose_;
  color_ = wp.color_;
  mass_ = wp.mass_;
  index_phy_ = wp.index_phy_;
  index_gui_ = wp.index_gui_;
  obj_guichanged_ = wp.obj_guichanged_;
  obj_phychanged_ = wp.obj_phychanged_;
  modifiable_gui_ = wp.modifiable_gui_;
  object_type_ = wp.object_type_;
}

spWaypoint::~spWaypoint() {}

void spWaypoint::SetLength(double length) {
  length_ = length;
  obj_guichanged_ = true;
}

double spWaypoint::GetLength() {
  return length_;
}


void spWaypoint::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_guichanged_ = true;
}

const spPose& spWaypoint::GetPose(){
  return pose_;
}

void spWaypoint::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spWaypoint::GetColor() {
  return color_;
}
