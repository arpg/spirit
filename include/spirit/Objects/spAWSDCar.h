#ifndef SP_AWSDCAR_H__
#define SP_AWSDCAR_H__

#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spBox.h>

class spAWSDCar : public spVehicle {
public:
  spAWSDCar(const spVehicleConstructionInfo& vehicle_info);
  ~spAWSDCar();
  void SetFrontSteeringAngle(double angle);
  void SetRearSteeringAngle(double angle);
  void SetEngineMaxVel(double vel);
  void SetEngineTorque(double torque);
  void SetSteeringServoMaxVel(double vel);
  void SetSteeringServoTorque(double torque);

private:
  void ApplyTransmissionDifferentialCoupling();
};

#endif  // SP_AWSDCAR_H__
