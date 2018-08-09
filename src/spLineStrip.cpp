#include <spirit/Objects/spLineStrip.h>

spLineStrip::spLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color) {
  mass_ = 0;
  color_ = color;
  pose_ = pose;
  SetLineStripPoints(linestrip_pts);
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::LINESTRIP;
}

spLineStrip::spLineStrip(const spPose& pose, const spCurve& curve,int num_points, const spColor& color) {
  mass_ = 0;
  color_ = color;
  pose_ = pose;
  SetLineStripPointsFromCurve(curve,num_points);
  index_gui_ = -1;
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

void spLineStrip::SetLineStripPointsFromCurve(const spCurve& curve, int num_pts){
  points_ = std::make_shared<spPoints3d>(num_pts);
  curve.GetPoints3d(*points_);
  obj_guichanged_ = true;
}

void spLineStrip::SetLineStripPoints(const spPoints3d& pts/*in their local coordinates*/) {
  points_ = std::make_shared<spPoints3d>(pts.size());
  *points_ = pts;
  obj_guichanged_ = true;
}

spPoints3d &spLineStrip::GetLineStripPoints() {
  return *points_;
}
