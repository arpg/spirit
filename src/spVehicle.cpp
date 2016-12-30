#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(const spVehicleConstructionInfo& vehicle_info) {
//  mass_ = vehicle_info.chassis_mass;
//  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
//    wheel_.push_back(std::make_shared<spWheel>(vehicle_info));
//    wheel_[ii]->SetChassisAnchor(vehicle_info.wheels_anchor[ii]);
//  }

//  pose_ = vehicle_info.pose;
//  MoveWheelsToAnchors();
//  color_ = vehicle_info.color;
//  cog_local_ = spPose::Identity();
//  cog_local_.translation() = vehicle_info.cog;
//  chassis_size_ = vehicle_info.chassis_size;
//  index_phy_ = -1;
//  index_gui_ = -1;
//  obj_phychanged_ = false;
//  obj_guichanged_ = false;
//  modifiable_gui_ = false;
//  obj_clamptosurface_ = false;
//  object_type_ = spObjectType::VEHICLE;
  friction = vehicle_info.chassis_friction;
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  mass_ = vehicle_info.chassis_mass;
  rot_vel = spRotVel(0,0,0);
  lin_vel = spLinVel(0,0,0);
  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>(vehicle_info));
    wheel_[ii]->SetChassisAnchor(vehicle_info.wheels_anchor[ii]);
  }

  SetPose(vehicle_info.pose);
  MoveWheelsToAnchors();
  SetColor(vehicle_info.color);
  cog_local_ = spPose::Identity();
  cog_local_.translation() = vehicle_info.cog;
  chassis_size_ = vehicle_info.chassis_size;
  object_type_ = vehicle_info.vehicle_type;
}

spVehicle::~spVehicle() {}

void spVehicle::SetPose(const spPose& pose) {
  // set chassis pose
  pose_ = pose;
  statevec_.head(3) = pose.translation();
  spRotation quat(pose.rotation());
  statevec_.segment(3,4) << quat.w(),quat.x(),quat.y(),quat.z();
  MoveWheelsToAnchors();
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

void spVehicle::MoveWheelsToAnchors(void) {
  for(int ii=0; ii<wheel_.size(); ii++) {
    spPose sp(spPose::Identity());
    spPose tr = pose_;
    sp.translate(pose_*(wheel_[ii]->GetChassisAnchor()));
//    sp.translate(pose_*(wheel_[ii]->GetChassisAnchor()+spTranslation(wheel_[ii]->GetWidth()/2,0,0)));
//    Eigen::AngleAxisd ang1(-SP_PI/2,Eigen::Vector3d::UnitY());
//    tr.rotate(ang1);
    sp.rotate(tr.rotation());
//    sp.rotate(pose_.rotation());
    wheel_[ii]->SetPose(sp);
  }
}

const spPose& spVehicle::GetPose() { return pose_; }

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
  mass_ = mass;
  obj_phychanged_ = true;
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

spWheel*spVehicle::GetWheel(int index)
{

  return wheel_[index].get();
}

void spVehicle::SetVelocity(const spVelocity& chassis_vel) {
  statevec_.tail<6>() = chassis_vel;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spStateVec& spVehicle::GetStateVecor() {
  return statevec_;
}

void spVehicle::SetClampToSurfaceFlag() {
  obj_clamptosurface_ = true;
}

const spLinVel& spVehicle::GetLinVel(){
  return lin_vel;
}

void spVehicle::SetLinVel(const spLinVel& vel) {
 lin_vel = vel;
}

const spRotVel& spVehicle::GetRotVel(){
  return rot_vel;
}

void spVehicle::SetRotVel(const spRotVel& vel) {
 rot_vel = vel;
}
