#ifndef SPIRIT_AXES_H_
#define SPIRIT_AXES_H_

#include <SceneGraph/SceneGraph.h>
#include <deque>
#include <spirit/objects/CommonObj.h>

class SpiritAxes : public SpiritCommonObj {
 public:
  SpiritAxes(SceneGraph::GLSceneGraph& graph);
  ~SpiritAxes();

  int AddObj(Eigen::Vector6d T_w_a);
  int NumOfObjs() { return vec_.size(); }
  int DelObj(int axis_num);

  void Clear() { for(size_t ii = 0; ii < NumOfObjs() ; ii++ ) { DelObj(ii); } }


 private:
  // glgraph to be updated
  SceneGraph::GLSceneGraph* glgraph_;

  // vector of objects
  std::vector<std::unique_ptr<SceneGraph::GLAxis>> vec_;

  // Axis properties
  float axis_scale_;
  bool axis_perceptable_;
};

#endif  // SPIRIT_AXES_H_
