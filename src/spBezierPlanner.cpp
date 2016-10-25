#include <spirit/Planners/spBezierPlanner.h>

spBezierPlanner::spBezierPlanner(){
  has_loop_ = false;
}

spBezierPlanner::~spBezierPlanner(){

}

void spBezierPlanner::AddWaypoint(const spWaypoint& waypoint, int index=-1) {
  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
  std::shared_ptr<spBezierCurve> new_curve = std::make_shared<spBezierCurve>();
  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
  std::vector<std::shared_ptr<spBezierCurve>>::iterator curve_it;
  waypoint_it = planpoint_vec_.begin();
  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
  curve_it = plancurve_vec_.begin();
  plancurve_vec_.insert(curve_it+index, new_curve);
  std::vector<bool>::iterator flag_it;
  flag_it = needs_curveupdate_vec_.begin();
  needs_curveupdate_vec_.insert(flag_it+index,true);
}

void spBezierPlanner::AddWaypoint(const spWaypoint& waypoint) {
  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
  std::shared_ptr<spBezierCurve> new_curve = std::make_shared<spBezierCurve>();
  planpoint_vec_.push_back(new_waypoint);
  plancurve_vec_.push_back(new_curve);
  needs_curveupdate_vec_.push_back(false);
  std::cout << "plan point added to index: " << plancurve_vec_.size()-1 << std::endl;
}

const spWaypoint& spBezierPlanner::GetWaypoint(int index) {
  return *planpoint_vec_[index];
}

void spBezierPlanner::UpdateWaypoint(const spWaypoint& planpoint,int index) {
//  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
//  std::shared_ptr<spBezierCurve> new_curve = std::make_shared<spBezierCurve>();
//  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
//  std::vector<std::shared_ptr<spBezierCurve>>::iterator curve_it;
//  waypoint_it = planpoint_vec_.begin();
//  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
//  curve_it = plancurve_vec_.begin();
//  plancurve_vec_.insert(curve_it+index, new_curve);

}

const spBezierCurve& spBezierPlanner::GetCurve(int index) {

}

void spBezierPlanner::RemoveWaypoint(int index_in_plan) {

}

int spBezierPlanner::GetNumWaypoints() {
  return planpoint_vec_.size();
}

void spBezierPlanner::HasLoop(bool has_loop) {
  has_loop_ = has_loop;
}

void spBezierPlanner::UpdateCurves() {
  for(int ii=0;ii<planpoint_vec_.size();ii++) {
    if(needs_curveupdate_vec_[ii]) {
      // solveBVP of that section
      std::cout << "needs update -> " << ii << std::endl;
    }
  }
}

void spBezierPlanner::SolveBVP() {

}
