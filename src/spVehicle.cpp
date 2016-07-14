#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(const spVehicleConstructionInfo& vehicle_info) {
  mass_ = vehicle_info.chassis_mass;
  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>(vehicle_info));
    wheel_[ii]->SetChassisAnchor(vehicle_info.wheels_anchor[ii]);
  }

  pose_ = vehicle_info.pose;
  MoveWheelsToAnchors();
  color_ = vehicle_info.color;
  cog_local_ = spPose::Identity();
  SetLocalCOG(vehicle_info.cog);
  chassis_size_ = vehicle_info.chassis_size;
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::VEHICLE;
  roll_influence_ = vehicle_info.roll_influence;
}

spVehicle::~spVehicle() {}

void spVehicle::SetPose(const spPose& pose) {
  // set chassis pose
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

void spVehicle::MoveWheelsToAnchors(void) {
  for(int ii=0; ii<wheel_.size(); ii++) {
    spPose sp(spPose::Identity());
    sp.translate(pose_*wheel_[ii]->GetChassisAnchor());
    sp.rotate(pose_.rotation());
    SetWheelOrigin(ii,sp);
  }
}

const spPose& spVehicle::GetPose() { return pose_; }

void spVehicle::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spPose& spVehicle::GetWheelOrigin(int index)
{
  return wheel_[index]->GetPose();
}

void spVehicle::SetWheelOrigin(int index, const spPose& pose)
{
  wheel_[index]->SetPose(pose);
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

double spVehicle::GetRollInfluence() {
  return roll_influence_;
}

void spVehicle::SetRollInfluence(double roll_inf) {
  roll_influence_ = roll_inf;
  obj_phychanged_ = true;
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

void spVehicle::SetChassisSize(const spBoxSize& dim) {
  chassis_size_ = dim;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spVehicle::GetLocalCOG(){
  return cog_local_;
}

const spPose& spVehicle::GetGlobalCOG(){
  return pose_*cog_local_;
}

void spVehicle::SetLocalCOG(const spTranslation& tr) {
  cog_local_.translation() = tr;
  obj_phychanged_ = true;
}

int spVehicle::GetNumberOfWheels()
{
  return wheel_.size();
}

spWheel*spVehicle::GetWheel(int index)
{
  return wheel_[index].get();
}
