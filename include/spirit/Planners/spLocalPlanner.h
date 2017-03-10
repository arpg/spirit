#ifndef LOCALPLANNER_H__
#define LOCALPLANNER_H__

#include <spirit/Planners/spTrajectory.h>
#include <spirit/CarSimFunctor.h>

class spLocalPlanner {
 public:
  spLocalPlanner(spTrajectory& initial_trajectory,const spVehicleConstructionInfo& vehicle_info);
  ~spLocalPlanner();
  void SolveLocalPlans();
  void CalcInitialPlans();

 private:
  spTrajectory& trajectory;
  spVehicleConstructionInfo vehicle_information;
  std::vector<std::shared_ptr<spCtrlPts3ord_3dof>> path_vec_;

};

#endif  // LOCALPLANNER_H__
