#include <spirit/Objects/spBezierCurve.h>

spBezierCurve::spBezierCurve() {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::BEZIER_CURVE;
  ctrl_pts_ = spBezierCtrlPoints::Zero();
}

spBezierCurve::~spBezierCurve() {}

void spBezierCurve::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_guichanged_ = true;
}

const spPose& spBezierCurve::GetPose() {
  return pose_;
}

void spBezierCurve::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spBezierCurve::GetColor() {
  return color_;
}

void spBezierCurve::SetControlPoints(const spBezierCtrlPoints& pts) {
  ctrl_pts_ = pts;
  obj_guichanged_ = true;
}

const spBezierCtrlPoints& spBezierCurve::GetControlPoints() {
  return ctrl_pts_;
}

void spBezierCurve::GetPoint(spPoint& point, double t) {
  point = (pow((1 - t), 3) * (spPoint)ctrl_pts_.col(0)) +
         (3 * pow((1 - t), 2) * t * (spPoint)ctrl_pts_.col(1)) +
         (3 * (1 - t) * pow(t, 2) * (spPoint)ctrl_pts_.col(2)) +
         (pow(t, 3) * (spPoint)ctrl_pts_.col(3));
  // calc global coordinates of the point
  point = pose_ * point;
}

void spBezierCurve::GetPoints(spPoints& pts_vec, int num_mid_pts) {
  int num_pts = num_mid_pts-1;
  for(int t=0;t<=num_pts;t++) {
    spPoint point;
    this->GetPoint(point,t*(1.0/num_pts));
//    std::shared_ptr<spPoint> point_ptr = std::make_shared<spPoint>(point);
    pts_vec.push_back(point);
  }
}



