#include <spirit/Planners/spCirclePlanner.h>

spCirclePlanner::spCirclePlanner(const spVehicleConstructionInfo& vehicle_info, double circle_radius, const spInputInstance2D& init_inputs, double init_linvel , unsigned int num_waypoints, Gui* gui):
  vehicle_parameters(vehicle_info), radius_(circle_radius), ss_input_(init_inputs), ss_linvel_(init_linvel), num_waypoints_(num_waypoints){
  weight_vec_ << 10, 10, 10, 0.1, 0.1, 0.1, 0.09, 0.09, 0.09, 0.1, 0.1, 0.1,0.1;
  gui_ = gui;
  travel_time_ = ((2*SP_PI*circle_radius)/init_linvel)/num_waypoints;
}

spCirclePlanner::~spCirclePlanner() {
}

void spCirclePlanner::SolvePlan() {

}

void spCirclePlanner::SolveInitialPlan(spTrajectory& trajectory) {
  for(int ii=0; ii<num_waypoints_; ii++) {
    double thetha = ii*(2*SP_PI)/num_waypoints_;
    double x = radius_*cos(thetha);
    double y = radius_*sin(thetha);
    spPose pose(spPose::Identity());
    pose.translate(spTranslation(x,y,0.07));
    Eigen::AngleAxisd rot(thetha+0.4, Eigen::Vector3d::UnitZ());
    pose.rotate(rot);

    trajectory.AddWaypoint(pose,ss_linvel_,spLinVel(0,1,0));
  }
}
