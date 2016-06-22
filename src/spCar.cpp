#include <spirit/Objects/spCar.h>

spCar::spCar(int num_wheels) {
  chassis_mass_ = 1;
  mass_ = chassis_mass_;
  for(int ii=0; ii<num_wheels; ii++) {
    wheel_.push_back(std::make_shared<spWheel>());
    mass_ += wheel_[ii]->GetMass();
  }
  if(num_wheels==4) {
    std::cout << "this called" << std::endl;
    SetWheelOrigin(0,spTranslation(1,1,-0.1));
    SetWheelOrigin(1,spTranslation(-1,1,-0.1));
    SetWheelOrigin(2,spTranslation(-1,-1,-0.1));
    SetWheelOrigin(3,spTranslation(1,-1,-0.1));
  }

  color_ = spColor(0, 0, 0);
  pose_ = spPose::Identity();
  cog_ = pose_.translation();
  cog_local_ = spTranslation(0,0,0);
  chassis_mass_ = 1;
  chassis_size_ = spBoxSize(1,1,1);
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
#warning "cog = pose for test here"
  cog_ = pose.translation();
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
  return wheel_[index]->origin;//->GetPose().translation();
}

void spCar::SetWheelOrigin(int index, const spTranslation& tr)
{
  wheel_[index]->origin = tr;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

double spCar::GetRollInfluence() {
  return roll_influence_;
}

void spCar::SetRollInfluence(double roll_inf) {
  roll_influence_ = roll_inf;
  obj_phychanged_ = true;
}

double spCar::GetChassisMass() {
  return chassis_mass_;
}

void spCar::SetChassisMass(double mass) {
  chassis_mass_ = mass;
  obj_phychanged_ = true;
}


const spBoxSize& spCar::GetChassisSize() {
  return chassis_size_;
}

void spCar::SetChassisSize(const spBoxSize& dim) {
  chassis_size_ = dim;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spTranslation& spCar::GetLocalCOG() {
#warning "implememt this"
  return cog_local_;
}

void spCar::SetLocalCOG(const spTranslation& tr) {
#warning "implememt this"
  obj_phychanged_ = true;
}

int spCar::GetNumberOfWheels()
{
  return wheel_.size();
}

spWheel*spCar::GetWheel(int index)
{
  return wheel_[index].get();
}
