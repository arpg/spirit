#ifndef SPMPC_H__
#define SPMPC_H__
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>
#include <spirit/spirit.h>
#include <spirit/MPCCostFunc.h>

class spMPC {
 public:
  spMPC(float horizon_duration);
  ~spMPC();
  void CalculateControls(const spTrajectory& ref_traj, const spState& curr_state, spCtrlPts2ord_2dof& controls);
  void SetHorizon(float horizon_duration);

 private:
  void FindClosestTrajPoint(int& index, const spTrajectory& ref_traj, const spState& curr_state);
  float horizon_;
};

#endif  // SPMPC_H__
