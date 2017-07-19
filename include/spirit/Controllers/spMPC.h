#ifndef SPMPC_H__
#define SPMPC_H__
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>
#include <spirit/spirit.h>
#include <spirit/MPCCostFunc.h>

class spMPC {
 public:
  spMPC();
  ~spMPC();
  void GetControlOutput(const spTrajectory& ref_traj, const spState& curr_state);
  void SetHorizon(int horizon);

 private:
  void FindClosestTrajPoint(int& index, const spTrajectory& ref_traj, const spState& base_state);
  int horizon_;
};

#endif  // SPMPC_H__
