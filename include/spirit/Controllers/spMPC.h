#ifndef SPMPC_H__
#define SPMPC_H__
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>
#include<spirit/Planners/spTrajectory.h>
#include <spirit/Controllers/MPCCostFunc.h>
#include <spirit/Controllers/MPCManRegCostFunc.h>
#include <spirit/ParamLimitLossFunc.h>
#include <spirit/LogBarrierLossFunc.h>

class spMPC {
 public:
  spMPC(const spVehicleConstructionInfo& car_params, float horizon_duration);
  ~spMPC();
  int CalculateControls(const spTrajectory& ref_traj, const spState& curr_state, spCtrlPts2ord_2dof& controls);
  int CircleManReg(const spState& curr_state, spCtrlPts2ord_2dof& controls, double radius, double tangent_vel);
  void SetHorizon(float horizon_duration);

 private:
  void FindClosestTrajPoint(int& closest_waypoint_index, int& closest_subindex, const spTrajectory& ref_traj, const spState& curr_state);
  void MinimizeMPCError(const spStateSeries& ref_states,const spState& current_state, spCtrlPts2ord_2dof& controls);
  void MinimizeMPCError(const Maneuver& maneuver, const spState& current_state, spCtrlPts2ord_2dof& controls);
  int horizon_;
  const spVehicleConstructionInfo& car_params_;
};

#endif  // SPMPC_H__
