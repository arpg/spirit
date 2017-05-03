#ifndef LOCALPLANNER_H__
#define LOCALPLANNER_H__

#include <spirit/Planners/spTrajectory.h>
#include <spirit/CarSimFunctor.h>
#include <spirit/VehicleCeresCostFunc.h>

class spLocalPlanner {
 public:
  spLocalPlanner(const spVehicleConstructionInfo& vehicle_info, Gui* gui = nullptr);
  ~spLocalPlanner();
  void SolveLocalPlan(spTrajectory& trajectory, int way_index, bool overwrite_endstate=false);
  void SolveLocalPlan(spTrajectory& trajectory);
  void SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint);
  void SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint, /*spState& final_state, */std::shared_ptr<spStateSeries> traj_states);
  void SolveInitialPlan(spTrajectory& trajectory, int way_index);
  void ShowSolverIterations(Gui* gui_);

private:
  spVehicleConstructionInfo vehicle_parameters;
  Gui* gui_;
//  std::vector<std::shared_ptr<spCtrlPts3ord_3dof>> path_vec_;
};

// This callback will be called at every iteration of ceres solver
//class SoverIterationCallback : public ceres::IterationCallback {
// public:
//  explicit SoverIterationCallback(Gui* gui, Objects* objects)
//      : gui_(gui), objects_(objects) {}

//  ~SoverIterationCallback() {}

//  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
//    std::cout << "callback called" << std::endl;
//    return ceres::SOLVER_CONTINUE;
//  }

// private:
//  Gui* gui_;
//  Objects* objects_;
//};

#endif  // LOCALPLANNER_H__
