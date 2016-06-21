#ifndef SP_WHEEL_H__
#define SP_WHEEL_H__

#include <spirit/Objects/spCommonObject.h>

class spWheel : public spCommonObject {
public:
  spWheel();
  ~spWheel();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  bool IsDynamic();

  double GetFriction();
  void SetFriction(double friction);

  double GetRadius();
  void SetRadius(double radius);

  double GetWidth();
  void SetWidth(double width);

  double GetSuspStiffness();
  void SetSuspStiffness(double stiffness);

  double GetSuspDamping();
  void SetSuspDamping(double damping);

  double GetSuspLowerLimit();
  void SetSuspLowerLimit(double limit);

  double GetSuspUpperLimit();
  void SetSuspUpperLimit(double limit);

  double GetSteeringLowerLimit();
  void SetSteeringLowerLimit(double limit);

  double GetSteeringUpperLimit();
  void SetSteeringUpperLimit(double limit);

  double GetSteeringMotorTargetVelocity();
  void SetSteeringMotorTargetVelocity(double velocity);

  double GetSteeringMotorTorque();
  void SetSteeringMotorTorque(double torque);

  bool GetHasDriveMotor();
  void SetHasDriveMotor(bool status);

  bool GetHasSteeringMotor();
  void SetHasSteeringMotor(bool status);

  double GetDriveMotorTargetVelocity();
  void SetDriveMotorTargetVelocity(double velocity);

  double GetDriveMotorTorque();
  void SetDriveMotorTorque(double torque);

  double GetMass();
  void SetMass(double mass);

  bool GetAirborneState();
  void SetAirborneState(bool status);


private:
  double friction_;  // unit less
  double width_;     // in meters
  double radius_;    // in meters
  double susp_damping_;
  double susp_stiffness_;
  double susp_lower_limit_;
  double susp_upper_limit_;
  double has_drive_motor_;
  double has_steering_motor_;
  double drive_motor_target_velocity_;
  double drive_motor_torque_;
  double steering_motor_torque_;
  double steering_motor_target_velocity_;
  double steering_lower_limit_;
  double steering_upper_limit_;
  bool airborne_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_WHEEL_H__
