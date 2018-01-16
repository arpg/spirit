#ifndef CIRCLEPLANNER_H__
#define CIRCLEPLANNER_H__

#include <spirit/Planners/spTrajectory.h>
#include <spirit/CarSimFunctor.h>
#include <spirit/Planners/VehicleCeresCostFunc.h>
#include <spirit/ParamLimitLossFunc.h>

class spCirclePlanner {
 public:
  spCirclePlanner(const spVehicleConstructionInfo& vehicle_info, double circle_radius, const spInputInstance2D& init_inputs, double init_linvel, unsigned int num_waypoints, Gui* gui = nullptr);
  ~spCirclePlanner();
  void SolvePlan();
  void SolveInitialPlan(spTrajectory& trajectory);

private:
  spVehicleConstructionInfo vehicle_parameters;
  spBVPWeightVec weight_vec_;
  double radius_;
  spInputInstance2D ss_input_;
  double ss_linvel_;
  unsigned int num_waypoints_;
  double travel_time_;
  Gui* gui_;
};

#endif  // CIRCLEPLANNER_H__
