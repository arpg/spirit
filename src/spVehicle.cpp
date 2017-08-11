#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(const spVehicleConstructionInfo& vehicle_info, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world) {
  mass_ = vehicle_info.chassis_mass;
  chassis_size_ = vehicle_info.chassis_size;
  // Create a bullet compound shape
//  chassis_compound = new btCompoundShape();
  chassis_compound_ = std::make_shared<btCompoundShape>();
  // add chassis as a box to compound shape
//  btCollisionShape* chassis_shape = new btBoxShape(btVector3(vehicle_info.chassis_size[0]/2.0,vehicle_info.chassis_size[1]/2.0,vehicle_info.chassis_size[2]/2.0)*WSCALE);
  chassis_shape_ = std::make_shared<btBoxShape>(btVector3(vehicle_info.chassis_size[0]/2.0,vehicle_info.chassis_size[1]/2.0,vehicle_info.chassis_size[2]/2.0)*WSCALE);
  // this transform is to put the cog in the right spot
  cog_local_ = spPose::Identity();
  cog_local_.translation() = vehicle_info.cog;
  btTransform tr;
  spPose2btTransform(cog_local_.inverse(),tr);
  chassis_compound_->addChildShape(tr,chassis_shape_.get());
  // create a rigidbody from compound shape and add it to world
  CreateRigidBody(vehicle_info.chassis_mass,cog_local_,chassis_compound_);
  rigid_body_->setFriction(vehicle_info.chassis_friction);
  int chassis_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
  dynamics_world->addRigidBody(rigid_body_.get(),BulletCollissionType::COL_CHASSIS,chassis_collides_with_);
  // set body velocities to zero
  rigid_body_->setLinearVelocity(btVector3(0,0,0));
  rigid_body_->setAngularVelocity(btVector3(0,0,0));
  // set damping to zero since we are moving in air
  rigid_body_->setDamping(0,0);
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);
  // create and add wheels
  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>(vehicle_info,ii,rigid_body_,dynamics_world));
    state_.InsertSubstate();
  }
  SetPose(vehicle_info.pose);
  MoveWheelsToAnchors(vehicle_info.pose);
  SetColor(vehicle_info.color);
  chassis_size_ = vehicle_info.chassis_size;
  object_type_ = vehicle_info.vehicle_type;
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
}

spVehicle::~spVehicle() {
//  delete chassis_compound->getChildShape(0);
//  delete chassis_compound;
}

void spVehicle::SetPose(const spPose& pose) {
  btTransform tr;
  spPose2btTransform(pose*GetLocalCOG(),tr);
  rigid_body_->setWorldTransform(tr);
//  statevec_.head(3) = pose.translation();
//  spRotation quat(pose.rotation());
//  statevec_.segment(3,4) << quat.w(),quat.x(),quat.y(),quat.z();

  MoveWheelsToAnchors(pose);
//  rigid_body_->clearForces();
//  GetWheel(0)->GetRigidbody()->clearForces();
//  GetWheel(1)->GetRigidbody()->clearForces();
//  GetWheel(2)->GetRigidbody()->clearForces();
//  GetWheel(3)->GetRigidbody()->clearForces();
  obj_guichanged_ = true;
}

void spVehicle::MoveWheelsToAnchors(const spPose& chasis_pose) {
  for(int ii=0; ii<wheel_.size(); ii++) {
    spPose sp(spPose::Identity());
    spTranslation anchor(wheel_[ii]->GetChassisAnchor());
    anchor[2] += 0.01;
    sp.translate(chasis_pose*(/*wheel_[ii]->GetChassisAnchor()+*/anchor));
//    sp.translate(pose_*(wheel_[ii]->GetChassisAnchor()+spTranslation(wheel_[ii]->GetWidth()/2,0,0)));
//    Eigen::AngleAxisd ang1(-SP_PI/2,Eigen::Vector3d::UnitY());
//    tr.rotate(ang1);
    sp.rotate(chasis_pose.rotation());
//    sp.rotate(pose_.rotation());
    wheel_[ii]->SetPose(sp);
  }
}

const spPose& spVehicle::GetPose() {
  btTransform2spPose(rigid_body_->getWorldTransform(),pose_);
  pose_ = pose_*GetLocalCOG().inverse();
  return pose_;
}

void spVehicle::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spVehicle::GetColor() {
  return color_;
}

const spPose& spVehicle::GetWheelOrigin(int index)
{
  return wheel_[index]->GetPose();
}

double spVehicle::GetChassisMass() {
  return mass_;
}

void spVehicle::SetChassisMass(double mass) {
  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
  chassis_compound_->calculateLocalInertia(mass,localInertia);
  rigid_body_->setMassProps(mass,localInertia);
  mass_ = mass;
}


const spBoxSize& spVehicle::GetChassisSize() {
  return chassis_size_;
}

const spPose& spVehicle::GetLocalCOG(){
  return cog_local_;
}

int spVehicle::GetNumberOfWheels()
{
  return wheel_.size();
}

std::shared_ptr<spWheel> spVehicle::GetWheel(int index)
{

  return wheel_[index];
}

void spVehicle::SetChassisLinearVelocity(const spLinVel& linear_vel) {
  rigid_body_->setLinearVelocity(btVector3(linear_vel[0],linear_vel[1],linear_vel[2]));
}

const spLinVel& spVehicle::GetChassisLinearVelocity() {
  ch_linvel_ = spLinVel(rigid_body_->getLinearVelocity()[0],rigid_body_->getLinearVelocity()[1],rigid_body_->getLinearVelocity()[2]);
  return ch_linvel_;
}

void spVehicle::SetChassisAngularVelocity(const spRotVel& angular_vel) {
  rigid_body_->setAngularVelocity(btVector3(angular_vel[0],angular_vel[1],angular_vel[2]));
}

const spRotVel& spVehicle::GetChassisAngularVelocity() {
  ch_rotvel_ = spRotVel(rigid_body_->getAngularVelocity()[0],rigid_body_->getAngularVelocity()[1],rigid_body_->getAngularVelocity()[2]);
  return ch_rotvel_;
}

const spState& spVehicle::GetState() {
//  std::shared_ptr<spState> state = std::make_shared<spState>();
  // bullet uses Euler XYZ(yaw,pitch,roll) convention for rotations
//  btTransform2spPose(rigid_body_->getWorldTransform(),state_.pose);
//  state_.pose = state_.pose*GetLocalCOG().inverse();
  state_.pose = GetPose();
  state_.linvel = GetChassisLinearVelocity();
  state_.rotvel = GetChassisAngularVelocity();
  for(int ii=0; ii<GetNumberOfWheels(); ii++) {
    state_.substate_vec[ii]->pose = GetWheel(ii)->GetPose();
    state_.substate_vec[ii]->linvel = GetWheel(ii)->GetLinVel();
    state_.substate_vec[ii]->rotvel = GetWheel(ii)->GetRotVel();
  }

//  state->w1 = std::make_shared<spState>();
//  state->w2 = std::make_shared<spState>();
//  state->w3 = std::make_shared<spState>();
//  state->w0->pose = GetWheel(0)->GetPose();
//  state->w0->linvel = GetWheel(0)->GetLinVel();
//  state->w0->rotvel = GetWheel(0)->GetRotVel();
//  state->w1->pose = GetWheel(1)->GetPose();
//  state->w1->linvel = GetWheel(1)->GetLinVel();
//  state->w1->rotvel = GetWheel(1)->GetRotVel();
//  state->w2->pose = GetWheel(2)->GetPose();
//  state->w2->linvel = GetWheel(2)->GetLinVel();
//  state->w2->rotvel = GetWheel(2)->GetRotVel();
//  state->w3->pose = GetWheel(3)->GetPose();
//  state->w3->linvel = GetWheel(3)->GetLinVel();
//  state->w3->rotvel = GetWheel(3)->GetRotVel();


//  spPose T_wheel_chassis = GetWheel(0)->GetPose().inverse()*GetPose();
//  double steering = T_wheel_chassis.rotation().eulerAngles(0,1,2)[2];
//  if((steering<0)&&(tan(steering)>0)){
//    steering += SP_PI;
//  } else if((steering>0)&&(tan(steering)<0)){
//    steering -= SP_PI;
//  }
//  state->front_steering = steering;

//  spPose T_wheel_chassis = GetWheel(0)->GetPose().inverse()*GetPose();
//  state->front_steering = T_wheel_chassis.rotation();
//  spPose T_wheel_chassis2 = GetWheel(3)->GetPose().inverse()*GetPose();
//  state->rear_steering = T_wheel_chassis2.rotation();

//  state->w1 = GetWheel(0)->GetPose();
//  state->w2 = GetWheel(1)->GetPose();
//  state->w3 = GetWheel(2)->GetPose();
//  state->w4 = GetWheel(3)->GetPose();

  return state_;
}

void spVehicle::SetState(const spState& state){
  if(state.substate_vec.size() != GetNumberOfWheels()) {
    SPERROR("state tree size mismatch.");
  }
  // TODO: when setting the pose we need to set suspention length aswell
  SetPose(state.pose);
//  std::cout << "linvel is " << state.linvel.transpose() << std::endl;
  SetChassisLinearVelocity(state.linvel);
  SetChassisAngularVelocity(state.rotvel);
//  for(int ii=0; ii<GetNumberOfWheels(); ii++) {
  for(int ii=0; ii<state.substate_vec.size(); ii++) {
    GetWheel(ii)->SetWheelSpeed(state.wheel_speeds[ii]);
    GetWheel(ii)->SetPose(state.substate_vec[ii]->pose);
    GetWheel(ii)->SetLinVel(state.substate_vec[ii]->linvel);
    GetWheel(ii)->SetAngularVel(state.substate_vec[ii]->rotvel);
  }
//  spPose T_wheel_chassis = GetWheel(0)->GetPose().inverse()*GetPose();
//  Eigen::AngleAxisd rollAngle(T_wheel_chassis.rotation().eulerAngles(0,1,2)[0], Eigen::Vector3d::UnitX());
//  Eigen::AngleAxisd pitchAngle(T_wheel_chassis.rotation().eulerAngles(0,1,2)[1], Eigen::Vector3d::UnitY());
//  Eigen::AngleAxisd yawAngle(state.front_steering, Eigen::Vector3d::UnitZ());
//  Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle ;
//  spPose p0(GetWheel(0)->GetPose());
//  spPose p3(GetWheel(3)->GetPose());
//  p0.rotation().Zero();
//  p0.rotate(q);
//  p3.rotation().Zero();
//  p3.rotate(q);
//  GetWheel(0)->SetPose(p0);
//  GetWheel(3)->SetPose(p3);

}

void spVehicle::SetClampToSurfaceFlag() {
  obj_clamptosurface_ = true;
}
