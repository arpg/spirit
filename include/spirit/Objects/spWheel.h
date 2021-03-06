#ifndef SP_WHEEL_H__
#define SP_WHEEL_H__

#include <spirit/Objects/spCommonObject.h>

class spWheel : public spCommonObject {
public:
  spWheel(const spVehicleConstructionInfo& vehicle_info, int wheel_index, std::shared_ptr<btRigidBody> chassis_body, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world);
  ~spWheel();
  void SetPose(const spPose& pose);
  void SetFriction(double friction);
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
  void SetAngle(double angle);
  double GetWheelSpeed();
  void SetWheelSpeed(double rps);
//  double GetSuspensionLength();
//  void SetSuspensionLength(double length);
  void SetAngularVel(const spRotVel& vel);
  void InitializeSteeringServoAngle(double angle);

private:
  spTranslation chassis_anchor;
  double width_;     // in meters
  double radius_;    // in meters
  const int drive_motor_axis = 3;
  const int steering_servo_axis = 5;
  spColor color_;
  // Use GetPose() whenever pose of object is required
  spLinVel linvel_;
  spRotVel rotvel_;
  std::shared_ptr<btHinge2Constraint> hinge_;
  std::shared_ptr<btCollisionShape> wheel_shape_;
  double friction_;
  double steering_lower_limit_;
  double steering_upper_limit_;
  double steering_max_velocity_;
  double steering_torque_;
  double steering_servo_target_angle_;
  double drive_motor_target_velocity_;
  double drive_motor_torque_;
  double wheel_speed_;
  bool drive_motor_;
  bool steering_servo_;

};

#endif  //  SP_WHEEL_H__
