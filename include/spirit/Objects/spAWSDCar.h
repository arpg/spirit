#ifndef SP_AWSDCAR_H__
#define SP_AWSDCAR_H__

#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spBox.h>

class spAWSDCar : public spVehicle {
public:
  spAWSDCar(const spVehicleConstructionInfo& vehicle_info);
  ~spAWSDCar();
  void SetFrontSteeringAngle(double angle);
  void SetBackSteeringAngle(double angle);
  void SetEngineTorque(double torque);
private:
  void ApplyTransmissionDifferentialCoupling();
};

#endif  // SP_AWSDCAR_H__
