#include <spirit/Controllers/spMPC.h>

spMPC::spMPC(float horizon_duration) {
  horizon_ = horizon_duration;
}

spMPC::~spMPC() {

}

void spMPC::CalculateControls(const spTrajectory& ref_traj, const spState& curr_state, spCtrlPts2ord_2dof& controls) {
  // find the closest trajectory point to current state
  int closest_index;
  FindClosestTrajPoint(closest_index,ref_traj,curr_state);

  // construct the MPC cost function
  SPERROREXIT("NOT IMPLEMENTED");
  // minimize the cost function
  SPERROREXIT("NOT IMPLEMENTED");
  // return controls found
}

void spMPC::SetHorizon(float horizon_duration) {
  horizon_ = horizon_duration;
}


void spMPC::FindClosestTrajPoint(int& index, const spTrajectory& ref_traj, const spState& curr_state) {
  for(int ii=0; ii<ref_traj.GetNumWaypoints(); ii++) {
    if(ref_traj.GetTravelDuration(ii) == -1) {
      SPERROREXIT("LocalPlanner solution doesn't exist");
    }
    for(int jj=0; jj<(int)(10*ref_traj.GetTravelDuration(ii)); jj++) {
      SPERROREXIT("NOT IMPLEMENTED");
    }
  }
}
