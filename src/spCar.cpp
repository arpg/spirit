#include <spirit/Objects/spCar.h>

spCar::spCar() {
  mass_ = 0;
  color_ = spColor(1, 1, 1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::BOX;
}

spCar::~spCar() {}

void spCar::SetParamsToDefaults() {
  chassis_mass_ = 1;
  wheel_friction_ = 100;
  engine_force_ = 0;
  breaking_force_ = 0;
  steering_angle_ = 0;
  steering_clamp_ = 0.3;
  wheel_radius_ = 0.5;
  wheel_width_ = 0.4;
  suspension_stiffness_ = 20;
  suspension_damping_ = 2.3;
  suspension_restLength_ = 0.6;
  roll_influence_ = 0.1;
}

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

bool spCar::IsDynamic() {
  if (mass_ > 0) {
    return true;
  } else {
    return false;
  }
}

void spCar::SetChassisMass(double mass) { chassis_mass_ = mass; }

void spCar::SetWheelFriction(double friction) { wheel_friction_ = friction; }

void spCar::SetEngineForce(double force) { engine_force_ = force; }

void spCar::SetBreakingForce(double force) { breaking_force_ = force; }

void spCar::SetSteeringAngle(double angle) { steering_angle_ = angle; }

void spCar::SetSteeringClamp(double angle) { steering_clamp_ = angle; }

void spCar::SetWheelRadius(double radius) { wheel_radius_ = radius; }

void spCar::SetWheelWidth(double width) { wheel_width_ = width; }

void spCar::SetSuspensionStiffness(double stiffness) {
  suspension_stiffness_ = stiffness;
}

void spCar::SetSuspensionDamping(double damping) {
  suspension_damping_ = damping;
}

void spCar::SetSuspensionRestLength(double length) {
  suspension_restLength_ = length;
}

void spCar::SetRollInfluence(double roll_influence) {
  roll_influence_ = roll_influence;
}

double spCar::GetChassisMass() { return mass_; }

double spCar::GetWheelFriction() { return wheel_friction_; }

double spCar::GetEngineForce() { return engine_force_; }

double spCar::GetBreakingForce() { return breaking_force_; }

double spCar::GetSteeringAngle() { return steering_angle_; }

double spCar::GetSteeringClamp() { return steering_clamp_; }

double spCar::GetWheelRadius() { return wheel_radius_; }

double spCar::GetWheelWidth() { return wheel_width_; }

double spCar::GetSuspensionStiffness() { return suspension_stiffness_; }

double spCar::GetSuspensionDamping() { return suspension_damping_; }

double spCar::GetSuspensionRestLength() { return suspension_restLength_; }

double spCar::GetRollInfluence() { return roll_influence_; }

void spCar::SetWheelFLPose(const spPose& wheel_pose) {
  pose_wheelFL_ = wheel_pose;
}

void spCar::SetWheelFRPose(const spPose& wheel_pose) {
  pose_wheelFR_ = wheel_pose;
}

void spCar::SetWheelBLPose(const spPose& wheel_pose) {
  pose_wheelBL_ = wheel_pose;
}

void spCar::SetWheelBRPose(const spPose& wheel_pose) {
  pose_wheelBR_ = wheel_pose;
}

spPose spCar::SetWheelFLPose() { return pose_wheelFL_; }

spPose spCar::SetWheelFRPose() { return pose_wheelFR_; }

spPose spCar::SetWheelBLPose() { return pose_wheelBL_; }

spPose spCar::SetWheelBRPose() { return pose_wheelBR_; }

void spCar::SetChassisSize(const spBoxSize& size) {
  chassis_size_ = size;
}

const spBoxSize& spCar::GetChassisSize() {
  return chassis_size_;
}

void spCar::SetWheelWeight(double mass) {
  wheel_weight_ = mass;
}

double spCar::GetWheelWeight() {
  return wheel_weight_;
}
