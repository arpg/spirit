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

  double GetSteeringServoLowerLimit();
  void SetSteeringServoLowerLimit(double limit);

  double GetSteeringServoUpperLimit();
  void SetSteeringServoUpperLimit(double limit);

  double GetSteeringServoMaxVelocity();
  void SetSteeringServoMaxVelocity(double velocity);

  double GetSteeringServoTorque();
  void SetSteeringServoTorque(double torque);

  bool GetHasDriveMotor();
  void SetHasDriveMotor(bool status);

  bool GetHasSteeringServo();
  void SetHasSteeringServo(bool status);

  double GetDriveMotorTargetVelocity();
  void SetDriveMotorTargetVelocity(double velocity);

  double GetDriveMotorTorque();
  void SetDriveMotorTorque(double torque);

  double GetMass();
  void SetMass(double mass);

  bool GetAirborneState();
  void SetAirborneState(bool status);

  const spTranslation& GetChassisAnchor();
  void SetChassisAnchor(const spTranslation anchor);

  double GetSteeringServoCurrentAngle();
  void SetSteeringServoCurrentAngle(double angle);

  double GetSteeringServoTargetAngle();
  void SetSteeringServoTargetAngle(double angle);

  int GetDriveMotorAxis();
  int GetSteeringServoAxis();

private:
  spTranslation chassis_anchor;
  double friction_;  // unit less
  double width_;     // in meters
  double radius_;    // in meters
  double susp_damping_;
  double susp_stiffness_;
  double susp_lower_limit_;
  double susp_upper_limit_;
  double has_drive_motor_;
  double has_steering_servo_;
  double drive_motor_target_velocity_;
  double drive_motor_torque_;
  double steering_servo_torque_;
  double steering_servo_max_velocity_;
  double steering_servo_lower_limit_;
  double steering_servo_upper_limit_;
  double steering_servo_target_angle_;
  double steering_servo_angle_;
  const int drive_motor_axis = 3;
  const int steering_servo_axis = 5;
  bool airborne_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_WHEEL_H__
