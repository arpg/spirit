#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(std::vector<spTranslation> wheel_anchor) {
  chassis_mass_ = 3;
  mass_ = chassis_mass_;
  for(int ii=0; ii<wheel_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>());
    wheel_[ii]->SetChassisAnchor(wheel_anchor[ii]);
    mass_ += wheel_[ii]->GetMass();
  }
  MoveWheelsToAnchors();
  color_ = spColor(0, 0, 0);
  pose_ = spPose::Identity();
  cog_ = pose_.translation();
  cog_local_ = spTranslation(0,0,0);
  chassis_size_ = spBoxSize(0.2,0.4,0.1);
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::VEHICLE;
  roll_influence_ = 0.1;
}

spVehicle::~spVehicle() {}

void spVehicle::SetPose(const spPose& pose) {
  // set chassis pose
  pose_ = pose;
#warning "cog = pose for test here"
//  cog_ = pose.translation();
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
  return chassis_mass_;
}

void spVehicle::SetChassisMass(double mass) {
  chassis_mass_ = mass;
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

const spTranslation& spVehicle::GetLocalCOG() {
#warning "implememt this"
  return cog_local_;
}

void spVehicle::SetLocalCOG(const spTranslation& tr) {
#warning "implememt this"
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
