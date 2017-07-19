#include <spirit/Controllers/spMPC.h>

spMPC::spMPC() {
  horizon_ = 5;
}

spMPC::~spMPC() {

}

void spMPC::GetControlOutput(const spTrajectory& ref_traj, const spState& curr_state) {

}

void spMPC::SetHorizon(int horizon) {
  horizon_ = horizon;
}

void spMPC::FindClosestTrajPoint(int& index, const spTrajectory& ref_traj, const spState& base_state) {
  for(int ii=0; ii<ref_traj.GetNumWaypoints(); ii++) {
    if(ref_traj.GetTravelDuration(ii) == -1) {
      SPERROREXIT("LocalPlanner solution doesn't exist");
    }
    for(int jj=0; jj<(int)(10*ref_traj.GetTravelDuration(ii)); jj++) {
//      spState state_diff = base_state -
    }
  }
}
