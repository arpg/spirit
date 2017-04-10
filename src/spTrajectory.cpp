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

spObjectHandle spTrajectory::AddWaypoint(const spPose& pose, double velocity, bool en_default_3ord3dof_traj) {
  spObjectHandle waypoint_handle = objects_.CreateWaypoint(pose,spColor(1, 1, 0));
  ((spWaypoint&)objects_.GetObject(waypoint_handle)).SetLinearVelocityNorm(velocity);
  gui_.AddObject(objects_.GetObject(waypoint_handle));
  waypoint_vec_.push_back(&(spWaypoint&)objects_.GetObject(waypoint_handle));
  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
  spCtrlPts3ord_3dof pts;
  has_def_traj_vec_.push_back(en_default_3ord3dof_traj);
//    pts.col(0) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
//    pts.col(1) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
//    pts.col(2) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
//    pts.col(3) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
//    new_curve->SetHermiteControlPoints(pts);
  spObjectHandle linestrip_handle = objects_.CreateLineStrip(spPose::Identity(),*new_curve,num_pts_per_curve,spColor(0.4, 0, 0));
  gui_.AddObject(objects_.GetObject(linestrip_handle));
  linestrip_handle_vec_.push_back(linestrip_handle);
//  trajectorystrip_vec_.push_back(&(spLineStrip&)objects_.GetObject(linestrip_handle));
  curve_vec_.push_back(new_curve);

  // create storage for command which will drive the vehicle from this waypoint to the next one
  std::shared_ptr<spCtrlPts2ord_2dof> cntrl_cmd = std::make_shared<spCtrlPts2ord_2dof>();

  InitControlCommand(cntrl_cmd.get());
  control_command_vec_.push_back(cntrl_cmd);
}

void spTrajectory::InitControlCommand(spCtrlPts2ord_2dof* cntrl_cmd){
  // for now set it to a zero angle steering and some reasonable acceleration
  // TODO(sina) : later I should come up with a better initialization based on waypoint locations
  cntrl_cmd->col(0) = Eigen::Vector2d(0,0);
  cntrl_cmd->col(1) = Eigen::Vector2d(0,0);
  cntrl_cmd->col(2) = Eigen::Vector2d(0,0);
}

spVehicleConstructionInfo* spTrajectory::GetVehicleInfo(int waypoint_index) {
SPERROREXIT("NOT IMPLEMENTED.");
}

void spTrajectory::SetTrajectoryPoints(int waypoint_index, const spPoints3d& traj_pts) {
  ((spLineStrip&)objects_.GetObject(linestrip_handle_vec_[waypoint_index])).SetLineStripPoints(traj_pts);
}

void spTrajectory::SetControls(int waypoint_index, const spCtrlPts2ord_2dof& control_command){
  *control_command_vec_[waypoint_index] = control_command;
}
spCtrlPts2ord_2dof& spTrajectory::GetControls(int waypoint_index) {
  if(waypoint_index > control_command_vec_.size()) {
    SPERROREXIT("Index out of bounds.");
  }
  return *control_command_vec_[waypoint_index];
}

spWaypoint& spTrajectory::GetWaypoint(unsigned int index) {
  if(index>waypoint_vec_.size()-1) {
    SPERROREXIT("Requested index doesn't exist.");
  }
  return *waypoint_vec_[index];
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

// Update curves between waypoints based on their poses
// this curve is constrainted between two waypoints and it doesn't include any of car dynamics.
void spTrajectory::UpdateCurves() {
  for(int ii=0;ii<waypoint_vec_.size();ii++) {
    if(has_def_traj_vec_[ii]) {
      spCtrlPts3ord_3dof pts;
      if(ii == waypoint_vec_.size()-1) {
        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
//        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*waypoint_vec_[ii]->GetLinearVelocity();
        pts.col(2) = waypoint_vec_[0]->GetPose().translation();
//        pts.col(3) = waypoint_vec_[0]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[0]->GetLength();
        pts.col(3) = waypoint_vec_[0]->GetPose().rotation()*waypoint_vec_[0]->GetLinearVelocity();
      } else {
        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
//        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*waypoint_vec_[ii]->GetLinearVelocity();
        pts.col(2) = waypoint_vec_[ii+1]->GetPose().translation();
//        pts.col(3) = waypoint_vec_[ii+1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii+1]->GetLength();
        pts.col(3) = waypoint_vec_[ii+1]->GetPose().rotation()*waypoint_vec_[ii+1]->GetLinearVelocity();
      }
      curve_vec_[ii]->SetHermiteControlPoints(pts);
      ((spLineStrip&)objects_.GetObject(linestrip_handle_vec_[ii])).SetLineStripPointsFromCurve(*curve_vec_[ii],num_pts_per_curve);
    }
  }
}
