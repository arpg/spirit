#include <spirit/Objects/spWheel.h>


spWheel::spWheel()
{
  friction_ = 1;
  width_ = 0.2;
  radius_ = 0.4;
  susp_damping_ = 2;
  susp_stiffness_ = 10;
  susp_lower_limit_ = -0.25;
  susp_upper_limit_ = 0.25;
  has_drive_motor_ = false;
  has_steering_motor_ = false;
  drive_motor_target_velocity_ = 0;
  drive_motor_torque_ = 0;
  steering_motor_torque_ = 0;
  steering_motor_target_velocity_ = 0;
  steering_lower_limit_ = -SP_PI/4;
  steering_upper_limit_ = SP_PI/4;
  airborne_ = false;
  color_ = spColor(0,0,0);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::WHEEL;

}

spWheel::~spWheel()
{

}

void spWheel::SetPose(const spPose& pose)
{
  pose_ = pose;
}

const spPose& spWheel::GetPose()
{
  return pose_;
}

void spWheel::SetColor(const spColor& color)
{
  color_ = color;
}

bool spWheel::IsDynamic()
{
  if(mass_>0)
    return true;
  else
    return false;
}

double spWheel::GetFriction()
{
  return friction_;
}

void spWheel::SetFriction(double friction)
{
  friction_ = friction;
}

double spWheel::GetRadius()
{
  return radius_;
}

void spWheel::SetRadius(double radius)
{
  radius_ = radius;
}

double spWheel::GetWidth()
{
  return width_;
}

void spWheel::SetWidth(double width)
{
  width_ = width;
}

double spWheel::GetSuspStiffness()
{
  return susp_stiffness_;
}

void spWheel::SetSuspStiffness(double stiffness)
{
  susp_stiffness_ = stiffness;
}

double spWheel::GetSuspDamping()
{
  return susp_damping_;
}

void spWheel::SetSuspDamping(double damping)
{
  susp_damping_ = damping;
}

double spWheel::GetSuspLowerLimit()
{
  return susp_lower_limit_;
}

void spWheel::SetSuspLowerLimit(double limit)
{
  susp_lower_limit_ = limit;
}

double spWheel::GetSuspUpperLimit()
{
  return susp_upper_limit_;
}

void spWheel::SetSuspUpperLimit(double limit)
{
  susp_upper_limit_ = limit;
}

double spWheel::GetSteeringLowerLimit()
{
  return steering_lower_limit_;
}

void spWheel::SetSteeringLowerLimit(double limit)
{
  steering_lower_limit_ = limit;
}

double spWheel::GetSteeringUpperLimit()
{
  return steering_upper_limit_;
}

void spWheel::SetSteeringUpperLimit(double limit)
{
  steering_upper_limit_ = limit;
}

double spWheel::GetSteeringMotorTargetVelocity()
{
  return steering_motor_target_velocity_;
}

void spWheel::SetSteeringMotorTargetVelocity(double velocity)
{
  steering_motor_target_velocity_ = velocity;
}

double spWheel::GetSteeringMotorTorque()
{
  return steering_motor_torque_;
}

void spWheel::SetSteeringMotorTorque(double torque)
{
  steering_motor_torque_ = torque;
}

bool spWheel::GetHasDriveMotor()
{
  return has_drive_motor_;
}

void spWheel::SetHasDriveMotor(bool status)
{
  has_drive_motor_ = status;
}

bool spWheel::GetHasSteeringMotor()
{
  return has_steering_motor_;
}

void spWheel::SetHasSteeringMotor(bool status)
{
  has_steering_motor_ = status;
}

double spWheel::GetDriveMotorTargetVelocity()
{
  return drive_motor_target_velocity_;
}

void spWheel::SetDriveMotorTargetVelocity(double velocity)
{
  drive_motor_target_velocity_ = velocity;
}

double spWheel::GetDriveMotorTorque()
{
  return drive_motor_torque_;
}

void spWheel::SetDriveMotorTorque(double torque)
{
  drive_motor_torque_ = torque;
}

double spWheel::GetMass()
{
  return mass_;
}

void spWheel::SetMass(double mass)
{
  mass_ = mass;
}

bool spWheel::GetAirborneState()
{
  return airborne_;
}

void spWheel::SetAirborneState(bool status)
{
  airborne_ = status;
}
