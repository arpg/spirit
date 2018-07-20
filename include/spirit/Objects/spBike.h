#ifndef SP_BIKE_H__
#define SP_BIKE_H__

#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spBox.h>

class spBike : public spVehicle {
public:
  spBike(const spVehicleConstructionInfo& vehicle_info, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world);
  ~spBike();
  void SetFrontSteeringAngle(double angle);
  void UpdateWheelFriction(double friction);
  void SetRearSteeringAngle(double angle);
  void SetEngineMaxVel(double vel);
  void SetEngineTorque(double torque);
//  void SetSteeringServoMaxVel(double vel);
//  void SetSteeringServoTorque(double torque);
//  void Initialize(const spPose& pose, const spLinVel& chassis_lin_vel, const spRotVel& chassis_rot_vel, double steering_angle, double engine_torque, double wheel_speeds);
private:
};

#endif  // SP_BIKE_H__
