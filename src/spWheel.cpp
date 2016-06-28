#include <spirit/Objects/spWheel.h>


spWheel::spWheel()
{
  friction_ = 1;
  width_ = 0.2;
  radius_ = 0.4;
  susp_damping_ = 0.2;
  susp_stiffness_ = 10;
  susp_upper_limit_ = 0.5;
  susp_lower_limit_ = 0.0;
  has_drive_motor_ = true;
  drive_motor_target_velocity_ = 1;
  drive_motor_torque_ = 10;

  has_steering_servo_ = true;
  steering_servo_torque_ = 50;
  steering_servo_max_velocity_ = 10;
  steering_servo_target_angle_ = 0;
  steering_servo_lower_limit_ = -SP_PI/4;
  steering_servo_upper_limit_ = SP_PI/4;

  airborne_ = false;
  color_ = spColor(0,0,0);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  object_type_ = spObjectType::WHEEL;
  mass_ = 1;
}

spWheel::~spWheel()
{

}

void spWheel::SetPose(const spPose& pose)
{
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spWheel::GetPose()
{
  return pose_;
}

void spWheel::SetColor(const spColor& color)
{
  color_ = color;
  obj_guichanged_ = true;
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
  obj_phychanged_ = true;
}

double spWheel::GetRadius()
{
  return radius_;
}

void spWheel::SetRadius(double radius)
{
  radius_ = radius;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

double spWheel::GetWidth()
{
  return width_;
}

void spWheel::SetWidth(double width)
{
  width_ = width;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

double spWheel::GetSuspStiffness()
{
  return susp_stiffness_;
}

void spWheel::SetSuspStiffness(double stiffness)
{
  susp_stiffness_ = stiffness;
  obj_phychanged_ = true;
}

double spWheel::GetSuspDamping()
{
  return susp_damping_;
}

void spWheel::SetSuspDamping(double damping)
{
  susp_damping_ = damping;
  obj_phychanged_ = true;
}

double spWheel::GetSuspLowerLimit()
{
  return susp_lower_limit_;
}

void spWheel::SetSuspLowerLimit(double limit)
{
  susp_lower_limit_ = limit;
  obj_phychanged_ = true;
}

double spWheel::GetSuspUpperLimit()
{
  return susp_upper_limit_;
}

void spWheel::SetSuspUpperLimit(double limit)
{
  susp_upper_limit_ = limit;
  obj_phychanged_ = true;
}

double spWheel::GetSteeringServoLowerLimit()
{
  return steering_servo_lower_limit_;
}

void spWheel::SetSteeringServoLowerLimit(double limit)
{
  steering_servo_lower_limit_ = limit;
  obj_phychanged_ = true;
}

double spWheel::GetSteeringServoUpperLimit()
{
  return steering_servo_upper_limit_;
  obj_phychanged_ = true;
}

void spWheel::SetSteeringServoUpperLimit(double limit)
{
  steering_servo_upper_limit_ = limit;
  obj_phychanged_ = true;
}

double spWheel::GetSteeringServoMaxVelocity()
{
  return steering_servo_max_velocity_;
}

void spWheel::SetSteeringServoMaxVelocity(double velocity)
{
  steering_servo_max_velocity_ = velocity;
  obj_phychanged_ = true;
}

double spWheel::GetSteeringServoTorque()
{
  return steering_servo_torque_;
}

void spWheel::SetSteeringServoTorque(double torque)
{
  steering_servo_torque_ = torque;
  obj_phychanged_ = true;
}

bool spWheel::GetHasDriveMotor()
{
  return has_drive_motor_;
}

void spWheel::SetHasDriveMotor(bool status)
{
  has_drive_motor_ = status;
  obj_phychanged_ = true;
}

bool spWheel::GetHasSteeringServo()
{
  return has_steering_servo_;
}

void spWheel::SetHasSteeringServo(bool status)
{
  has_steering_servo_ = status;
  obj_phychanged_ = true;
}

double spWheel::GetDriveMotorTargetVelocity()
{
  return drive_motor_target_velocity_;
}

void spWheel::SetDriveMotorTargetVelocity(double velocity)
{
  drive_motor_target_velocity_ = velocity;
  obj_phychanged_ = true;
}

double spWheel::GetDriveMotorTorque()
{
  return drive_motor_torque_;
}

void spWheel::SetDriveMotorTorque(double torque)
{
  drive_motor_torque_ = torque;
  obj_phychanged_ = true;
}

double spWheel::GetMass()
{
  return mass_;
}

void spWheel::SetMass(double mass)
{
  mass_ = mass;
  obj_phychanged_ = true;
}

bool spWheel::GetAirborneState()
{
  return airborne_;
}

void spWheel::SetAirborneState(bool status)
{
  airborne_ = status;
}

const Eigen::Vector3d&spWheel::GetChassisAnchor()
{
  return chassis_anchor;
}

void spWheel::SetChassisAnchor(const Eigen::Vector3d anchor)
{
  chassis_anchor = anchor;
}

double spWheel::GetSteeringServoCurrentAngle()
{
  return steering_servo_angle_;
}

void spWheel::SetSteeringServoCurrentAngle(double angle)
{
  steering_servo_angle_ = angle;
}

double spWheel::GetSteeringServoTargetAngle()
{
  return steering_servo_target_angle_;
}

void spWheel::SetSteeringServoTargetAngle(double angle)
{
  steering_servo_target_angle_ = angle;
}

int spWheel::GetDriveMotorAxis()
{
  return drive_motor_axis;
}

int spWheel::GetSteeringServoAxis()
{
  return steering_servo_axis;
}
