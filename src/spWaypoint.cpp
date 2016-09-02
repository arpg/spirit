#include <spirit/Objects/spWaypoint.h>

spWaypoint::spWaypoint() {
  mass_ = 0;
  color_ = spColor(1,1,0);
  pose_ = spPose::Identity();
  length_ = 1;
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = true;
  object_type_ = spObjectType::WAYPOINT;
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
