#ifndef SP_BEZIERPLANNER_H__
#define SP_BEZIERPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>
#include <spirit/Objects.h>
#include <thread>
#include <spirit/Gui.h>

// Local planner includes a set of

class spBezierPlanner {
public:
  spBezierPlanner(Gui* gui);
  ~spBezierPlanner();

  void AddWaypoint(const spWaypoint& waypoint, unsigned int index);
  void AddWaypoint(const spWaypoint& waypoint);
  const spWaypoint& GetWaypoint(unsigned int index);
  void UpdateWaypoint(const spWaypoint& planpoint,unsigned int index);
  const spCurve& GetCurve(unsigned int index);
  void RemoveWaypoint(unsigned int index_in_plan);
  void IsLoop(bool is_loop);
  int GetNumWaypoints();
  void UpdateCurves();
  void CalcJacobian(spPlannerJacobian& jacobian, const spCtrlPts3ord_2dof& cntrl_vars, unsigned int num_sim_steps, double sim_step_size, const spPose& init_pose, double fd_delta);

private:
//  Physics jac_physics_;
  Gui*     jac_gui_;
  Objects jac_objects_;
  int jac_car_handle;

  void SolveBVP();
  bool is_loop_;
  spCtrlPts3ord_3dof ctrl_pts_;

  std::vector<std::shared_ptr<spWaypoint>> planpoint_vec_;
  std::vector<std::shared_ptr<spCurve>> plancurve_vec_;
  std::vector<bool> needs_curveupdate_vec_;

};

#endif  //  SP_BEZIERPLANNER_H__
