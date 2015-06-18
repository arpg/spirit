#ifndef SPIRIT_WAYPOINTS_H_
#define SPIRIT_WAYPOINTS_H_

#include <SceneGraph/SceneGraph.h>
#include <deque>
#include <spirit/objects/CommonObj.h>

class SpiritWaypoints : public SpiritCommonObj {
 public:
  SpiritWaypoints(SceneGraph::GLSceneGraph& graph);
  ~SpiritWaypoints();

  int AddObj(Eigen::Vector6d T_w_a);
  int NumOfObjs() { return vec_.size(); }
  int DelObj(int axis_num);

 private:
  // glgraph to be updated
  SceneGraph::GLSceneGraph* glgraph_;

  // vector of objects
  std::vector<std::unique_ptr<SceneGraph::GLWayPoint>> vec_;

  // waypoint properties
  SceneGraph::GLWayPoint test_waypoint_;
};

#endif  // SPIRIT_WAYPOINTS_H_
