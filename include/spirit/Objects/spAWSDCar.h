#ifndef SP_AWSDCAR_H__
#define SP_AWSDCAR_H__

#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spBox.h>

class spAWSDCar : public spVehicle {
public:
  spAWSDCar(const spVehicleConstructionInfo& vehicle_info, btDiscreteDynamicsWorld* dyn_world, btAlignedObjectArray<btCollisionShape*>& col_shapes);
  ~spAWSDCar();
  void SetFrontSteeringAngle(double angle);
  void SetRearSteeringAngle(double angle);
  void SetEngineMaxVel(double vel);
  void SetEngineTorque(double torque);
  void SetSteeringServoMaxVel(double vel);
  void SetSteeringServoTorque(double torque);
  void SetState();
  void GetState();
  void Initialize(spPose pose, spLinVel chassis_lin_vel, spRotVel chassis_rot_vel, double steering_angle, double engine_torque, double wheel_speeds);
private:
  void ApplyTransmissionDifferentialCoupling();
};

#endif  // SP_AWSDCAR_H__
