#include <spirit/Objects/spCar.h>

spCar::spCar(int num_wheels) {
  chassis_mass_ = 1;
  mass_ = chassis_mass_;
  for(int ii=0; ii<num_wheels; ii++) {
    wheel_.push_back(std::make_shared<spWheel>());
    mass_ += wheel_[ii]->GetMass();
  }

  color_ = spColor(0, 0, 0);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::CAR;
  roll_influence_ = 0.1;
}

spCar::~spCar() {}

void spCar::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spCar::GetPose() { return pose_; }

void spCar::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spTranslation& spCar::GetWheelOrigin(int index)
{
  return wheel_[index]->GetPose().translation();
}

void spCar::SetWheelOrigin(int index, const spTranslation& tr)
{
  spPose pose = wheel_[index]->GetPose();
  pose.translate(tr);
  wheel_[index]->SetPose(pose);
}

double spCar::GetRollInfluence() {
  return roll_influence_;
}

void spCar::SetRollInfluence(double roll_inf) {
  roll_influence_ = roll_inf;
}

double spCar::GetChassisMass() {
  return chassis_mass_;
}

void spCar::SetChassisMass(double mass) {
  chassis_mass_ = mass;
}


const spBoxSize& spCar::GetChassisSize() {
  return chassis_size_;
}

void spCar::SetChassisSize(const spBoxSize& dim) {
  chassis_size_ = dim;
}

const spTranslation& spCar::GetLocalCOG() {
#warning "implememt this"
  return cog_local_;
}

void spCar::SetLocalCOG(const spTranslation& tr) {
#warning "implememt this"
}

const spPose& spCar::GetChassisPose() {
  return chassis_pose_;
}

int spCar::GetNumberOfWheels()
{
  return wheel_.size();
}

spWheel*spCar::GetWheel(int index)
{
  return wheel_[index].get();
}
