#ifndef LOCALPLANNER_H__
#define LOCALPLANNER_H__

#include <spirit/Planners/spTrajectory.h>
#include <spirit/CarSimFunctor.h>
#include <spirit/Planners/VehicleCeresCostFunc.h>
#include <spirit/ParamLimitLossFunc.h>

class spLocalPlanner {
 public:
  spLocalPlanner(const spVehicleConstructionInfo& vehicle_info, bool overwrite_endstate=false, Gui* gui = nullptr);
  ~spLocalPlanner();
  double SolveLocalPlan(spTrajectory& trajectory, int way_index);
  void SolveLocalPlan(spTrajectory& trajectory);
  double SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint);
  double SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint, /*spState& final_state, */std::shared_ptr<spStateSeries> traj_states);
  void SolveInitialPlan(spTrajectory& trajectory, int way_index);
  void ShowSolverIterations(Gui* gui_);
  void SetCostWeight(const spBVPWeightVec& vec);

private:
  spVehicleConstructionInfo vehicle_parameters;
  spBVPWeightVec weight_vec_;
  bool overwrite_endstate_;
  Gui* gui_;
};

#endif  // LOCALPLANNER_H__
