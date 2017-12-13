#ifndef SP_AWSDCAR_H__
#define SP_AWSDCAR_H__

#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spBox.h>

class spAWSDCar : public spVehicle {
public:
  spAWSDCar(const spVehicleConstructionInfo& vehicle_info, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world);
  ~spAWSDCar();
  void SetFrontSteeringAngle(double angle);
  void SetRearSteeringAngle(double angle);
  void SetEngineMaxVel(double vel);
  void SetEngineTorque(double torque);
//  void SetSteeringServoMaxVel(double vel);
//  void SetSteeringServoTorque(double torque);
//  void Initialize(const spPose& pose, const spLinVel& chassis_lin_vel, const spRotVel& chassis_rot_vel, double steering_angle, double engine_torque, double wheel_speeds);
private:
};

#endif  // SP_AWSDCAR_H__
