#include <spirit/objects/SpiritWaypoints.h>

SpiritWaypoints::SpiritWaypoints(SceneGraph::GLSceneGraph& graph)
    : glgraph_(&graph) {}

SpiritWaypoints::~SpiritWaypoints() { delete glgraph_; }

int SpiritWaypoints::AddObj(Eigen::Vector6d T_w_a) {
  std::unique_ptr<SceneGraph::GLWayPoint> waypoint(new SceneGraph::GLWayPoint);
  vec_.push_back(std::move(waypoint));
  vec_.back()->SetPose(T_w_a[0], T_w_a[1], T_w_a[2], T_w_a[3], T_w_a[4], T_w_a[5]);
  glgraph_->AddChild(vec_.back().get());
  return vec_.size();
}

int SpiritWaypoints::DelObj(int axis_num) {
  if (axis_num < vec_.size()) {
    it_ = vec_.begin();
    glgraph_->RemoveChild(it_[axis_num].get());
    vec_.erase(it_ + axis_num);
    return vec_.size();
  } else {
    return -1;
  }
}
