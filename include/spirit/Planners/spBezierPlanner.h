#ifndef SP_BEZIERPLANNER_H__
#define SP_BEZIERPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spBezierCurve.h>

// Local planner includes a set of

class spBezierPlanner {
public:
  spBezierPlanner();
  ~spBezierPlanner();

  void AddWaypoint(const spWaypoint& waypoint, int index);
  void AddWaypoint(const spWaypoint& waypoint);
  const spWaypoint& GetWaypoint(int index);
  void UpdateWaypoint(const spWaypoint& planpoint,int index);
  const spBezierCurve& GetCurve(int index);
  void RemoveWaypoint(int index_in_plan);
  void HasLoop(bool has_loop);
  int GetNumWaypoints();
  void UpdateCurves();

private:
  void SolveBVP();
  bool has_loop_;
  spBezierCtrlPoints ctrl_pts_;
  std::vector<std::shared_ptr<spWaypoint>> planpoint_vec_;
  std::vector<std::shared_ptr<spBezierCurve>> plancurve_vec_;
  std::vector<bool> needs_curveupdate_vec_;
};

#endif  //  SP_BEZIERPLANNER_H__
