#include <spirit/Planners/spLocalPlanner.h>

spLocalPlanner::spLocalPlanner(spTrajectory& initial_trajectory, const spVehicleConstructionInfo& vehicle_info): trajectory(initial_trajectory), vehicle_information(vehicle_info) {
}

spLocalPlanner::~spLocalPlanner() {
}

void spLocalPlanner::SolveLocalPlans() {

}

void spLocalPlanner::CalcInitialPlans() {
  for(int ii=0; ii<1/*trajectory.GetNumWaypoints()*/; ii++) {
    spState a;

//    CarSimFunctor sim(vehicle_information,);
//    sim(0,10,0.1,trajectory.GetControls(ii),0,-1);
//    trajectory.SetTrajectoryPoints(ii,sim.GetTrajectoryPoints());
  }
}

