#ifndef SP_BEZIERPLANNER_H__
#define SP_BEZIERPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>
#include <spirit/Objects.h>
#include <spirit/Physics.h>
#include <thread>
// Local planner includes a set of

class spBezierPlanner {
public:
  spBezierPlanner();
  ~spBezierPlanner();

  void AddWaypoint(const spWaypoint& waypoint, unsigned int index);
  void AddWaypoint(const spWaypoint& waypoint);
  const spWaypoint& GetWaypoint(unsigned int index);
  void UpdateWaypoint(const spWaypoint& planpoint,unsigned int index);
  const spCurve& GetCurve(unsigned int index);
  void RemoveWaypoint(unsigned int index_in_plan);
  void HasLoop(bool has_loop);
  int GetNumWaypoints();
  void UpdateCurves();

private:
  void CalcJacobian(spPlannerJacobian& jacobian, const spCtrlPts3ord_2dof& cntrl_vars, unsigned int num_sim_steps, double sim_step_size, spPose& init_pose, double fd_delta);
  Physics jac_physics_;
  Objects jac_objects_;
  int jac_car_handle;

  void SolveBVP();
  bool has_loop_;
  spCtrlPts3ord_3dof ctrl_pts_;

  std::vector<std::shared_ptr<spWaypoint>> planpoint_vec_;
  std::vector<std::shared_ptr<spCurve>> plancurve_vec_;
  std::vector<bool> needs_curveupdate_vec_;
};

#endif  //  SP_BEZIERPLANNER_H__
