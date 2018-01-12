#include <spirit/Objects/spWaypoint.h>

spWaypoint::spWaypoint(const spPose& pose, const spColor& color) {
  mass_ = 0;
  color_ = color;
  pose_ = pose;
  linveldir_ = spLinVel(0,1,0);
  rotvel_ = spRotVel(0,0,0);
  linvelnorm_ = 1;
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = true;
  object_type_ = spObjectType::WAYPOINT;
}

spWaypoint::spWaypoint(const spWaypoint& wp) {
  pose_ = wp.pose_;
  color_ = wp.color_;
  mass_ = wp.mass_;
  index_gui_ = wp.index_gui_;
  obj_guichanged_ = wp.obj_guichanged_;
  modifiable_gui_ = wp.modifiable_gui_;
  object_type_ = wp.object_type_;
  linvelnorm_ = wp.linvelnorm_;
}

spWaypoint::~spWaypoint() {}

void spWaypoint::SetLinearVelocityDirection(const spLinVel& linvel_dir) {
  linveldir_ = linvel_dir;
  linveldir_.normalize();
}

void spWaypoint::SetLinearVelocityNorm(double velocity) {
  linvelnorm_ = velocity;
  obj_guichanged_ = true;
}

double spWaypoint::GetLinearVelocityNorm() {
  return linvelnorm_;
}

void spWaypoint::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_guichanged_ = true;
}

const spPose& spWaypoint::GetPose() const {
  return pose_;
}

const spPose& spWaypoint::GetPose() {
  return pose_;
}

void spWaypoint::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spLinVel spWaypoint::GetLinearVelocityInWorld() const {
  spLinVel linvel;
  linvel = spLinVel(pose_.rotation()*linveldir_);
//  linvel.normalize();
  linvel *= linvelnorm_;
  return linvel;
}

const spColor& spWaypoint::GetColor() {
  return color_;
}

void spWaypoint::SetRotVel(const spRotVel& rotvel) {
  rotvel_ = rotvel;
}

const spRotVel& spWaypoint::GetRotVel() {
  return rotvel_;
}
