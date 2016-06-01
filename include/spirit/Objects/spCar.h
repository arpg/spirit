#ifndef SP_CAR_H__
#define SP_CAR_H__

#include <spirit/Objects/spCommonObject.h>

class spCar : public spCommonObject {
 public:
  spCar();
  ~spCar();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  bool IsDynamic();

  void SetChassisMass(double mass);
  void SetWheelFriction(double friction);
  void SetEngineForce(double force);
  void SetBreakingForce(double force);
  void SetSteeringAngle(double angle);
  void SetSteeringClamp(double angle);
  void SetWheelRadius(double radius);
  void SetWheelWidth(double width);
  void SetSuspensionStiffness(double stiffness);
  void SetSuspensionDamping(double damping);
  void SetSuspensionRestLength(double length);
  void SetRollInfluence(double roll_influence);

  void SetParamsToDefaults();

  double GetChassisMass();
  double GetWheelFriction();
  double GetEngineForce();
  double GetBreakingForce();
  double GetSteeringAngle();
  double GetSteeringClamp();
  double GetWheelRadius();
  double GetWheelWidth();
  double GetSuspensionStiffness();
  double GetSuspensionDamping();
  double GetSuspensionRestLength();
  double GetRollInfluence();

 private:
  spPose pose_;  // this pose will represent geometric center of the car
  spColor color_;

  // Car parameters
  double chassis_mass_;    // in kg
  double wheel_friction_;  // unit less
  double engine_force_;    // in newton
  double breaking_force_;  // in newton
  double steering_angle_;  // in radians
  double steering_clamp_;  // in radians
  double wheel_radius_;    // in meters
  double wheel_width_;     // in meters
  double suspension_stiffness_;
  double suspension_damping_;
  double suspension_restLength_;
  double roll_influence_;
};

#endif  //  SP_BOX_H__
