#ifndef SP_WHEEL_H__
#define SP_WHEEL_H__

#include <spirit/Objects/spCommonObject.h>

class spWheel : public spCommonObject {
public:
  spWheel(const spVehicleConstructionInfo& vehicle_info, int wheel_index, btRigidBody* chassis_body, btDiscreteDynamicsWorld* dynamics_world);
  ~spWheel();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();
  double GetRadius();
  double GetWidth();
  void SetSteeringServoLowerLimit(double limit);
  void SetSteeringServoUpperLimit(double limit);
  void SetSteeringServoMaxVelocity(double velocity);
  void SetSteeringServoTorque(double torque);
  void SetDriveMotorTargetVelocity(double velocity);
  void SetDriveMotorTorque(double torque);
  const spTranslation& GetChassisAnchor();
  double GetSteeringServoCurrentAngle();
  void SetSteeringServoTargetAngle(double angle);
  void EnableDriveMotor(bool status);
  void EnableSteeringServo(bool status);
  const spRotVel& GetRotVel();
  const spLinVel& GetLinVel();
  void SetLinVel(const spLinVel& vel);
//  void SetAngle(double angle);
  double GetWheelSpeed();
  void SetWheelSpeed(double rps);

private:
  spTranslation chassis_anchor;
  double width_;     // in meters
  double radius_;    // in meters
  const int drive_motor_axis = 3;
  const int steering_servo_axis = 5;
  spColor color_;
  btHinge2Constraint* hinge_;
};

#endif  //  SP_WHEEL_H__
