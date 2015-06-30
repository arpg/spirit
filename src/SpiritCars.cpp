#include <spirit/objects/SpiritCars.h>

SpiritCars::SpiritCars(SceneGraph::GLSceneGraph &graph) : glgraph_(&graph) {
  num_of_worlds_ = 1;
}

SpiritCars::~SpiritCars() {
  delete glgraph_;
  delete collision_shape_;
}

int SpiritCars::AddObj(Eigen::Vector6d T_w_c) {
  if (collision_shape_ == nullptr) {
    std::cerr << "Error : SpiritCars collision shape has not been set"
              << std::endl;
    return -1;
  }

  std::unique_ptr<SpiritCar> new_car(new SpiritCar);
  // Initialize both the terrain and car in bullet

  VehicleState state;
  // set initial pose of the car
  state.m_dTwv = Sophus::SE3d(SceneGraph::GLCart2T(T_w_c));

  new_car->physicscar.Init(collision_shape_, dMin_, dMax_, default_params_map_,
                          num_of_worlds_);
//  BulletCarModel tmp;
//  tmp.SetState();
  new_car->physicscar.SetState(num_of_worlds_-1,state);
  new_car->glcar.Init(eMesh);
  new_car->glcar.SetCarScale(
      Eigen::Vector3d(default_params_map_[CarParameters::WheelBase],
                      default_params_map_[CarParameters::Width],
                      default_params_map_[CarParameters::WheelBase]));
  new_car->carlinesegments.SetColor(SceneGraph::GLColor(0.0f, 0.0f, 1.0f));
  vec_.push_back(std::move(new_car));

  // add to scenegraph
  glgraph_->AddChild(&vec_.back()->glcar);
  for (size_t ii = 0; ii < vec_.back()->glcar.GetWheels().size(); ii++) {
    glgraph_->AddChild(vec_.back()->glcar.GetWheels()[ii]);
  }
  // get cars current wheels state and update them
  state.UpdateWheels(vec_.back()->physicscar.GetWheelTransforms(0));
  // update cars state
  this->SetCarState(vec_.size() - 1, state, false);

  return vec_.size();
}

int SpiritCars::NumOfObjs() { return vec_.size(); }

int SpiritCars::DelObj(int car_num) {
  if (car_num < vec_.size()) {
    glgraph_->RemoveChild(&vec_[car_num]->glcar);
    for (size_t ii = 0; ii < vec_.back()->glcar.GetWheels().size(); ii++) {
      glgraph_->RemoveChild(vec_[car_num]->glcar.GetWheels()[ii]);
    }
    vec_.erase(vec_.begin() + car_num);
    return vec_.size();
  } else {
    std::cerr << " - Requested object to delete doese not exist." << std::endl;
    return -1;
  }
}

void SpiritCars::InitializeMap(btCollisionShape *col_shape) {
  collision_shape_ = col_shape;
}

void SpiritCars::InitCarParams(std::string params_file_str) {
  CarParameters::LoadFromFile(params_file_str, default_params_map_);
}

void SpiritCars::SetCarState(const int &id, const VehicleState &state,
                             bool bAddToTrajectory /* = false */) {
  //    std::unique_lock<std::mutex> lock(*pCar);

  Sophus::SE3d state_aug = state.m_dTwv;
  state_aug.translation() -= GetBasisVector(state_aug, 2) * 0.05;
  vec_[id]->glcar.SetPose(state_aug.matrix());

  Sophus::SE3d axisPose = state.m_dTwv;
  VehicleState::AlignWithVelocityVector(axisPose, state.m_dV);
  vec_[id]->glaxis.SetPose(axisPose.matrix());
  vec_[id]->glaxis.SetAxisSize(state.m_dV.norm());

  for (size_t ii = 0; ii < state.m_vWheelStates.size(); ii++) {
    Sophus::SE3d T = state.m_vWheelStates[ii];
    vec_[id]->glcar.SetRelativeWheelPose(ii, T);
  }

  // add this pose to the trajectory of the car if required
  if (bAddToTrajectory) {
    vec_[id]->carlinesegments.AddVertex(state.m_dTwv.translation());
  }
}

void SpiritCars::SetCarVisibility(const int &id, const bool &bVisible) {
  vec_[id]->glcar.SetVisible(bVisible);
  for (size_t ii = 0; ii < vec_[id]->glcar.GetWheels().size(); ii++) {
    vec_[id]->glcar.GetWheels()[ii]->SetVisible(bVisible);
  }
}

void SpiritCars::UpdateGuiFromPhysics(const int& world_id) {
  VehicleState state;
  for (size_t ii=0; ii<vec_.size(); ii++) {
    //   update opengl state with physics state
    ControlCommand cmd;
    cmd.m_dForce = 0.5;
    cmd.m_dT = 0.5;
    vec_[ii]->physicscar.UpdateState(0,cmd,1,true,true);
    vec_[ii]->physicscar.GetVehicleState(world_id,state);
    state.UpdateWheels(vec_[ii]->physicscar.GetWheelTransforms(0));
    this->SetCarState(ii, state, false);
  }
}
