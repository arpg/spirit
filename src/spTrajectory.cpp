#include <spirit/Planners/spTrajectory.h>

spTrajectory::spTrajectory(Gui& gui, Objects& objects) : gui_(gui),objects_(objects) {
  is_loop_ = false;
}

spTrajectory::~spTrajectory(){
}

void spTrajectory::AddWaypoint(const spWaypoint& waypoint, unsigned int index) {
  SPERROREXIT("This function has not been implemented yet");
//  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
//  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
//  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
//  std::vector<std::shared_ptr<spCurve>>::iterator curve_it;
//  waypoint_it = planpoint_vec_.begin();
//  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
//  curve_it = plancurve_vec_.begin();
//  plancurve_vec_.insert(curve_it+index, new_curve);
//  std::vector<bool>::iterator flag_it;
//  flag_it = needs_curveupdate_vec_.begin();
//  needs_curveupdate_vec_.insert(flag_it+index,true);
}

spObjectHandle spTrajectory::AddWaypoint(const spPose& pose) {
  spObjectHandle waypoint_handle = objects_.CreateWaypoint(pose,spColor(1, 1, 0));
  gui_.AddObject(objects_.GetObject(waypoint_handle));
  waypoint_vec_.push_back(&(spWaypoint&)objects_.GetObject(waypoint_handle));
  spCtrlPts3ord_3dof pts;
  pts.col(0) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
  pts.col(1) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
  pts.col(2) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
  pts.col(3) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
  spPoints3d line_pts(num_pts_per_curve);
  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
  new_curve->SetHermiteControlPoints(pts);
  new_curve->GetPoints3d(line_pts);
  spObjectHandle linestrip_handle = objects_.CreateLineStrip(spPose::Identity(),line_pts,spColor(0.4, 0, 0));
  gui_.AddObject(objects_.GetObject(linestrip_handle));
  linestrip_vec_.push_back(&(spLineStrip&)objects_.GetObject(linestrip_handle));

  curve_vec_.push_back(new_curve);
  needs_update_vec_.push_back(true);
}

const spWaypoint& spTrajectory::GetWaypoint(unsigned int index) {
  SPERROREXIT("This function has not been implemented yet");
//  if(index>planpoint_vec_.size()-1) {
//    SPERROREXIT("Requested index doesn't exist.");
//  }
//  return *planpoint_vec_[index];
}

void spTrajectory::UpdateWaypoint(const spWaypoint& planpoint,unsigned int index) {
  SPERROREXIT("This function has not been implemented yet");
//  std::shared_ptr<spWaypoint> new_waypoint = std::make_shared<spWaypoint>(waypoint);
//  std::shared_ptr<spBezierCurve> new_curve = std::make_shared<spBezierCurve>();
//  std::vector<std::shared_ptr<spWaypoint>>::iterator waypoint_it;
//  std::vector<std::shared_ptr<spBezierCurve>>::iterator curve_it;
//  waypoint_it = planpoint_vec_.begin();
//  planpoint_vec_.insert(waypoint_it+index, new_waypoint);
//  curve_it = plancurve_vec_.begin();
//  plancurve_vec_.insert(curve_it+index, new_curve);
}

const spCurve& spTrajectory::GetCurve(unsigned int index) {
  SPERROREXIT("This function has not been implemented yet");

}

void spTrajectory::RemoveWaypoint(unsigned int index_in_plan) {
  SPERROREXIT("This function has not been implemented yet");

}

int spTrajectory::GetNumWaypoints() {
  return waypoint_vec_.size();
}

void spTrajectory::IsLoop(bool is_loop) {
  is_loop_ = is_loop;
}

// Update curves between waypoints if their corresponding
void spTrajectory::UpdateCurves() {
  for(int ii=0;ii<waypoint_vec_.size();ii++) {
//    if(needs_update_vec_[ii]) {
      spCtrlPts3ord_3dof pts;
      if(ii == waypoint_vec_.size()-1) {
        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
        pts.col(2) = waypoint_vec_[0]->GetPose().translation();
        pts.col(3) = waypoint_vec_[0]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[0]->GetLength();
      } else {
        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
        pts.col(2) = waypoint_vec_[ii+1]->GetPose().translation();
        pts.col(3) = waypoint_vec_[ii+1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii+1]->GetLength();
      }
      curve_vec_[ii]->SetHermiteControlPoints(pts);
      spPoints3d line_pts(num_pts_per_curve);
      curve_vec_[ii]->GetPoints3d(line_pts);
      linestrip_vec_[ii]->SetLineStripPoints(line_pts);
//    }
  }
}
