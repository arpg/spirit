#include <spirit/objects/SpiritAxes.h>

SpiritAxes::SpiritAxes(SceneGraph::GLSceneGraph& graph) : glgraph_(&graph) {
  axis_scale_ = 0.5;
  axis_perceptable_ = false;
}

SpiritAxes::~SpiritAxes() { delete glgraph_; }

int SpiritAxes::AddObj(Eigen::Vector6d T_w_a) {
  std::unique_ptr<SceneGraph::GLAxis> glaxis(new SceneGraph::GLAxis);
  vec_.push_back(std::move(glaxis));
  vec_.back()->SetPose(T_w_a[0], T_w_a[1], T_w_a[2], T_w_a[3], T_w_a[4], T_w_a[5]);
  vec_.back()->SetScale(axis_scale_);
  vec_.back()->SetPerceptable(axis_perceptable_);
  glgraph_->AddChild(vec_.back().get());
  return vec_.size();
}

int SpiritAxes::DelObj(int axis_num) {
  if (axis_num < vec_.size()) {
    glgraph_->RemoveChild(vec_[axis_num].get());
    vec_.erase(vec_.begin() + axis_num);
    return vec_.size();
  } else {
    std::cerr << " - Requested object to delete does not exist." << std::endl;
    return -1;
  }
}
