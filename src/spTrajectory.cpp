#include <spirit/Planners/spTrajectory.h>

spTrajectory::spTrajectory(Gui& gui, std::shared_ptr<Objects> &objects) : gui_(gui), objects_(objects) {
  objects_ = std::make_shared<Objects>(spPhyEngineType::PHY_NONE);
  is_loop_ = false;
}

spTrajectory::~spTrajectory(){
}

spObjectHandle spTrajectory::AddWaypoint(const spPose& pose, double velocity/*, bool en_default_3ord3dof_traj*/) {
  spObjectHandle waypoint_handle = objects_->CreateWaypoint(pose,spColor(1, 1, 0));
  ((spWaypoint&)objects_->GetObject(waypoint_handle)).SetLinearVelocityNorm(velocity);
  gui_.AddObject(objects_->GetObject(waypoint_handle));
  waypoint_vec_.push_back(&(spWaypoint&)objects_->GetObject(waypoint_handle));
//  std::shared_ptr<spCurve> new_curve = std::make_shared<spCurve>(3,3);
//  spCtrlPts3ord_3dof pts;
//  has_def_traj_vec_.push_back(en_default_3ord3dof_traj);
////    pts.col(0) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
////    pts.col(1) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
////    pts.col(2) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().translation();
////    pts.col(3) = waypoint_vec_[waypoint_vec_.size()-1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[waypoint_vec_.size()-1]->GetLength();
////    new_curve->SetHermiteControlPoints(pts);
//  spObjectHandle linestrip_handle = objects_.CreateLineStrip(spPose::Identity(),*new_curve,10,spColor(0.4, 0, 0));
  spPoints3d points;
  spObjectHandle linestrip_handle = objects_->CreateLineStrip(spPose::Identity(),points,spColor(0.4, 0, 0));
  gui_.AddObject(objects_->GetObject(linestrip_handle));
  linestrip_handle_vec_.push_back(linestrip_handle);
////  trajectorystrip_vec_.push_back(&(spLineStrip&)objects_.GetObject(linestrip_handle));
//  curve_vec_.push_back(new_curve);

  // create storage for command which will drive the vehicle from this waypoint to the next one
  std::shared_ptr<spCtrlPts2ord_2dof> cntrl_cmd = std::make_shared<spCtrlPts2ord_2dof>();

  // InitTravelDuration
  travel_duration_vec_.push_back(-1);
  InitControlCommand(cntrl_cmd.get());
  control_command_vec_.push_back(cntrl_cmd);
  stateseries_vec_.push_back(nullptr);
}

void spTrajectory::InitControlCommand(spCtrlPts2ord_2dof* cntrl_cmd){
  // for now set it to a zero angle steering and some reasonable acceleration
  // TODO(sina) : later I should come up with a better initialization based on relative waypoint locations
    cntrl_cmd->col(0) = Eigen::Vector2d(-0.0,10);
    cntrl_cmd->col(1) = Eigen::Vector2d(-0.0,10);
    cntrl_cmd->col(2) = Eigen::Vector2d(-0.0,10);
}

void spTrajectory::PlaybackTrajectoryOnGUI(const spVehicleConstructionInfo& vehicle_params, int waypoint_index, double playback_ratio, int max_num_steps) {
  if(GetTravelDuration(waypoint_index)==-1) {
    SPERROR("Trajectory not planned yet!");
    return;
  }
  spObjectHandle car_handle = objects_->CreateVehicle(vehicle_params);
  gui_.AddObject(objects_->GetObject(car_handle));
  spStateSeries& stateseries = *(stateseries_vec_[waypoint_index]);
  int num_steps = stateseries.size();
  if(max_num_steps != -1) {
    num_steps = max_num_steps;
  }
  for (int ii = 0; ii < num_steps-1; ++ii) {
    ((spAWSDCar&)objects_->GetObject(car_handle)).SetState(*stateseries[ii]);
    for(int jj=0; jj<(int)(10.0/playback_ratio);jj++) {
      gui_.Iterate(objects_);
      spGeneralTools::Delay_ms(7);
    }
  }
  gui_.RemoveObject(objects_->GetObject(car_handle));
  objects_->RemoveObj(car_handle);
}

//void spTrajectory::SetTrajectoryPoints(int waypoint_index, const spPoints3d& traj_pts) {
//  ((spLineStrip&)objects_.GetObject(linestrip_handle_vec_[waypoint_index])).SetLineStripPoints(traj_pts);
//}

void spTrajectory::SetTrajectoryStateSeries(int waypoint_index, std::shared_ptr<spStateSeries> state_series ) {
  // set state series
  stateseries_vec_[waypoint_index] = state_series;
  // set new trajectory positions for linestrip
  spPoints3d points;
  for(int ii=0; ii<state_series->size(); ii++) {
    points.push_back((*state_series)[ii]->pose.translation());
  }
  ((spLineStrip&)objects_->GetObject(linestrip_handle_vec_[waypoint_index])).SetLineStripPoints(points);
}

std::shared_ptr<spStateSeries> spTrajectory::GetTrajectoryStateSeries(int waypoint_index) const {
  return stateseries_vec_[waypoint_index];
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

spWaypoint& spTrajectory::GetWaypoint(unsigned int index) const {
  if(index>waypoint_vec_.size()-1) {
    SPERROREXIT("Requested index doesn't exist.");
  }
  return *waypoint_vec_[index];
}

int spTrajectory::GetNumWaypoints() const{
  return waypoint_vec_.size();
}

void spTrajectory::IsLoop(bool is_loop) {
  is_loop_ = is_loop;
}

bool spTrajectory::IsLoop() const {
  return is_loop_;
}

void spTrajectory::SetTravelDuration(int waypoint_index, double travel_time) {
  travel_duration_vec_[waypoint_index] = travel_time;
}

double spTrajectory::GetTravelDuration(int waypoint_index) const {
  return travel_duration_vec_[waypoint_index];
}




// Update curves between waypoints based on their poses
// this curve is constrainted between two waypoints and it doesn't include any of car dynamics.
//void spTrajectory::UpdateCurves() {
//  for(int ii=0;ii<waypoint_vec_.size();ii++) {
//    if(has_def_traj_vec_[ii]) {
//      spCtrlPts3ord_3dof pts;
//      if(ii == waypoint_vec_.size()-1) {
//        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
////        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
//        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*waypoint_vec_[ii]->GetLinearVelocity();
//        pts.col(2) = waypoint_vec_[0]->GetPose().translation();
////        pts.col(3) = waypoint_vec_[0]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[0]->GetLength();
//        pts.col(3) = waypoint_vec_[0]->GetPose().rotation()*waypoint_vec_[0]->GetLinearVelocity();
//      } else {
//        pts.col(0) = waypoint_vec_[ii]->GetPose().translation();
////        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii]->GetLength();
//        pts.col(1) = waypoint_vec_[ii]->GetPose().rotation()*waypoint_vec_[ii]->GetLinearVelocity();
//        pts.col(2) = waypoint_vec_[ii+1]->GetPose().translation();
////        pts.col(3) = waypoint_vec_[ii+1]->GetPose().rotation()*spTranslation(1,0,0)*waypoint_vec_[ii+1]->GetLength();
//        pts.col(3) = waypoint_vec_[ii+1]->GetPose().rotation()*waypoint_vec_[ii+1]->GetLinearVelocity();
//      }
//      curve_vec_[ii]->SetHermiteControlPoints(pts);
//      ((spLineStrip&)objects_.GetObject(linestrip_handle_vec_[ii])).SetLineStripPointsFromCurve(*curve_vec_[ii],num_pts_per_curve);
//    }
//  }
//}
