#ifndef SP_CAR_H__
#define SP_CAR_H__

#include <spirit/Objects/spCommonObject.h>
#include <vector>

// 1 - Geometric center of the chassis box will be the center of car object and
// any local transformation will be with respect to coordinate system in this
// center.
// 2 - every transformation is in global coordinates unless it has name "Local"
// in its fucntion name.
class spCar : public spCommonObject {
 public:
  spCar(int num_wheels);
  ~spCar();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  bool IsDynamic();

  const spPose& GetChassisPose();

  void SetParamsToDefaults();

  int GetNumberOfWheels();

  double GetWheelFriction(int index);
  void SetWheelFriction(int index, double friction);

  double GetWheelRadius(int index);
  void SetWheelRadius(int index, double radius);

  double GetWheelWidth(int index);
  void SetWheelWidth(int index, double width);

  const spTranslation& GetWheelOrigin(int index);

  double GetWheelSuspStiffness(int index);
  void SetWheelSuspStiffness(int index, double stiffness);

  double GetWheelSuspDamping(int index);
  void SetWheelSuspDamping(int index, double damping);

  double GetWheelSuspLowerLimit(int index);
  void SetWheelSuspLowerLimit(int index, double limit);

  double GetWheelSuspUpperLimit(int index);
  void SetWheelSuspUpperLimit(int index, double limit);

  double GetWheelSteeringLowerLimit(int index);
  void SetWheelSteeringLowerLimit(int index, double limit);

  double GetWheelSteeringUpperLimit(int index);
  void SetWheelSteeringUpperLimit(int index, double limit);

  double GetWheelSteeringMotorTargetVelocity(int index);
  void SetWheelSteeringMotorTargetVelocity(int index, double velocity);

  double GetWheelSteeringMotorTorque(int index);
  void SetWheelSteeringMotorTorque(int index, double torque);

  bool GetWheelHasDriveMotor(int index);
  void SetWheelHasDriveMotor(int index, bool status);

  bool GetWheelHasSteeringMotor(int index);
  void SetWheelHasSteeringMotor(int index, bool status);

  double GetWheelDriveMotorTargetVelocity(int index);
  void SetWheelDriveMotorTargetVelocity(int index, double velocity);

  double GetWheelDriveMotorTorque(int index);
  void SetWheelDriveMotorTorque(int index, double torque);

  double GetWheelMass(int index);
  void SetWheelMass(int index, double mass);

  double GetRollInfluence();
  void SetRollInfluence(double roll_inf);

  double GetChassisMass();
  void SetChassisMass(double mass);

  const spBoxSize& GetChassisSize();
  void SetChassisSize(const spBoxSize& dim);

  const spTranslation& GetLocalCOG();
  void SetLocalCOG(const spTranslation& tr);

  struct spWheel {
    spTranslation origin_tr;
    double friction;  // unit less
    double width;     // in meters
    double radius;    // in meters
    double mass;    //  in Kg
    double susp_damping;
    double susp_stiffness;
    double susp_lower_limit;
    double susp_upper_limit;
    double has_drive_motor;
    double has_steering_motor;
    double drive_motor_target_velocity;
    double drive_motor_torque;
    double steering_motor_torque;
    double steering_motor_target_velocity;
    double steering_lower_limit;
    double steering_upper_limit;
    double breaking_force_;  // in newton
    bool airborne;
  };

 protected:
  void  SetNumberOfWheels(int num);
  void SetChassisPose(const spPose& pose);
  void SetWheelOrigin(int index, const spTranslation& tr);

 private:
  spPose pose_;        // this pose will represent geometric center of the car
  spTranslation cog_;  // center of gravity
  spColor color_;
  spBoxSize chassis_size_;
  spPose chassis_pose_;
  spTranslation cog_local_;
  std::vector<spWheel> wheel;
  // Car parameters
  double chassis_mass_;  // in kg
  double roll_influence_;
};

#endif  //  SP_BOX_H__
