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

  void SetWheelFriction(double friction);
  void SetEngineForce(double force);
  void SetBreakingForce(double force);
  void SetSteeringAngle(double angle);
  void SetSteeringClamp(double angle);
  void SetWheelRadius(double radius);
  void SetWheelWidth(double width);
  void SetWheelFLPose(const spPose& wheel_pose);
  void SetWheelFRPose(const spPose& wheel_pose);
  void SetWheelBLPose(const spPose& wheel_pose);
  void SetWheelBRPose(const spPose& wheel_pose);
  void SetSuspensionStiffness(double stiffness);
  void SetSuspensionDamping(double damping);
  void SetSuspensionRestLength(double length);
  void SetRollInfluence(double roll_influence);
  void SetChassisSize(const spBoxSize& size);
  void SetChassisMass(double mass);
  void SetParamsToDefaults();
  void SetWheelWeight(double mass);

  double GetWheelFriction();
  double GetEngineForce();
  double GetBreakingForce();
  double GetSteeringAngle();
  double GetSteeringClamp();
  double GetWheelRadius();
  double GetWheelWidth();
  spPose SetWheelFLPose();
  spPose SetWheelFRPose();
  spPose SetWheelBLPose();
  spPose SetWheelBRPose();
  double GetSuspensionStiffness();
  double GetSuspensionDamping();
  double GetSuspensionRestLength();
  double GetRollInfluence();
  const spBoxSize& GetChassisSize();
  double GetChassisMass();
  double GetWheelWeight();

 private:
  spPose pose_;  // this pose will represent geometric center of the car
  spColor color_;
  spBoxSize chassis_size_;
  spPose pose_wheelFL_;
  spPose pose_wheelFR_;
  spPose pose_wheelBL_;
  spPose pose_wheelBR_;

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
  double wheel_weight_;
};

#endif  //  SP_BOX_H__
