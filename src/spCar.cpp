#include <spirit/Objects/spCar.h>

spCar::spCar(int num_wheels) {
  SetNumberOfWheels(num_wheels);
  mass_ = 0;
  color_ = spColor(1, 1, 1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::CAR;
}

spCar::~spCar() {}

void spCar::SetParamsToDefaults() {
  chassis_mass_ = 1;
  roll_influence_ = 0.1;
//  wheel_friction_ = 100;
//  engine_force_ = 0;
//  breaking_force_ = 0;
//  steering_angle_ = 0;
//  steering_clamp_ = 0.3;
//  wheel_radius_ = 0.5;
//  wheel_width_ = 0.4;
//  suspension_stiffness_ = 20;
//  suspension_damping_ = 2.3;
//  suspension_restLength_ = 0.6;
  for(int ii=0; ii<GetNumberOfWheels(); ii++) {
    wheel[ii].friction = 100;
    wheel[ii].width = 0.2;
    wheel[ii].radius = 0.5;
    wheel[ii].mass = 0.5;
    wheel[ii].susp_damping = 0.1;
    wheel[ii].susp_stiffness = 0.1;
    wheel[ii].susp_lower_limit = -0.25;
    wheel[ii].susp_upper_limit = 0.25;
    wheel[ii].has_drive_motor = true;
    wheel[ii].has_steering_motor = true;
    wheel[ii].drive_motor_target_velocity = 1000;
    wheel[ii].drive_motor_torque = 10;
    wheel[ii].steering_motor_target_velocity = 0;
    wheel[ii].steering_upper_limit = SP_PI/10;
    wheel[ii].steering_lower_limit = -SP_PI/10;
  }
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

void spCar::SetNumberOfWheels(int num) {
  wheel.resize(num);
}

int spCar::GetNumberOfWheels() {
  return wheel.size();
}

double spCar::GetWheelFriction(int index)
{
  return wheel[index].friction;
}

void spCar::SetWheelFriction(int index, double friction)
{
  wheel[index].friction = friction;
}

double spCar::GetWheelRadius(int index)
{
  return wheel[index].radius;
}

void spCar::SetWheelRadius(int index, double radius)
{
  wheel[index].radius = radius;
}

double spCar::GetWheelWidth(int index)
{
  return wheel[index].width;
}

void spCar::SetWheelWidth(int index, double width)
{
  wheel[index].width = width;
}

const spTranslation& spCar::GetWheelOrigin(int index)
{
  return wheel[index].origin_tr;
}

double spCar::GetWheelSuspStiffness(int index)
{
  return wheel[index].susp_stiffness;
}

void spCar::SetWheelSuspStiffness(int index, double stiffness) {
  wheel[index].susp_stiffness = stiffness;
}

double spCar::GetWheelSuspDamping(int index) {
  return wheel[index].susp_damping;
}

void spCar::SetWheelSuspDamping(int index, double damping) {
  wheel[index].susp_damping = damping;
}

double spCar::GetWheelSuspLowerLimit(int index) {
  return wheel[index].susp_lower_limit;
}

void spCar::SetWheelSuspLowerLimit(int index, double limit) {
  wheel[index].susp_lower_limit = limit;
}

double spCar::GetWheelSuspUpperLimit(int index) {
  return wheel[index].susp_upper_limit;
}

void spCar::SetWheelSuspUpperLimit(int index, double limit) {
  wheel[index].susp_upper_limit = limit;
}

double spCar::GetWheelSteeringLowerLimit(int index) {
  return wheel[index].steering_lower_limit;
}

void spCar::SetWheelSteeringLowerLimit(int index, double limit) {
  wheel[index].steering_lower_limit = limit;
}

double spCar::GetWheelSteeringUpperLimit(int index) {
  return wheel[index].steering_upper_limit;
}

void spCar::SetWheelSteeringUpperLimit(int index, double limit) {
  wheel[index].steering_upper_limit = limit;
}

double spCar::GetWheelSteeringMotorTargetVelocity(int index) {
  return wheel[index].steering_motor_target_velocity;
}

void spCar::SetWheelSteeringMotorTargetVelocity(int index, double velocity) {
  wheel[index].steering_motor_target_velocity = velocity;
}

double spCar::GetWheelSteeringMotorTorque(int index) {
  return wheel[index].steering_motor_torque;
}

void spCar::SetWheelSteeringMotorTorque(int index, double torque) {
  wheel[index].steering_motor_torque = torque;
}

bool spCar::GetWheelHasDriveMotor(int index) {
  return wheel[index].has_drive_motor;
}

void spCar::SetWheelHasDriveMotor(int index, bool status) {
  wheel[index].has_drive_motor = status;
}

bool spCar::GetWheelHasSteeringMotor(int index) {
  return wheel[index].has_steering_motor;
}

void spCar::SetWheelHasSteeringMotor(int index, bool status) {
  wheel[index].has_steering_motor = status;
}

double spCar::GetWheelDriveMotorTargetVelocity(int index) {
  return wheel[index].drive_motor_target_velocity;
}

void spCar::SetWheelDriveMotorTargetVelocity(int index, double velocity) {
  wheel[index].drive_motor_target_velocity = velocity;
}

double spCar::GetWheelDriveMotorTorque(int index) {
  return wheel[index].drive_motor_torque;
}

void spCar::SetWheelDriveMotorTorque(int index, double torque) {
  wheel[index].drive_motor_torque = torque;
}

double spCar::GetWheelMass(int index) {
  return wheel[index].mass;
}

void spCar::SetWheelMass(int index, double mass) {
  wheel[index].mass = mass;
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

void spCar::SetWheelOrigin(int index, const spTranslation& tr)
{
  wheel[index].origin_tr = tr;
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
