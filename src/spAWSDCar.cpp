#include <spirit/Objects/spAWSDCar.h>

spAWSDCar::spAWSDCar(const spVehicleConstructionInfo& vehicle_info):spVehicle(vehicle_info) {
  for(int ii=0;ii<4;ii++)
//  int ii =0;
  {
    spWheel* wheel = GetWheel(ii);
    wheel->SetHasSteeringServo(true);
    wheel->SetSteeringServoMaxVelocity(0.2);
    wheel->SetSteeringServoTorque(10);
//    wheel->SetSteeringServoTargetAngle(SP_PI/2);
    wheel->SetSteeringServoTargetAngle(0);
    wheel->SetHasDriveMotor(true);
    wheel->SetDriveMotorTargetVelocity(20);
    wheel->SetDriveMotorTorque(0.91);
  }
}

spAWSDCar::~spAWSDCar(){
}

void spAWSDCar::SetFrontSteeringAngle(double angle) {
  GetWheel(0)->SetSteeringServoTargetAngle(angle);
  GetWheel(3)->SetSteeringServoTargetAngle(angle);
}

void spAWSDCar::SetBackSteeringAngle(double angle) {
  GetWheel(1)->SetSteeringServoTargetAngle(angle);
  GetWheel(2)->SetSteeringServoTargetAngle(angle);
}

void spAWSDCar::SetMotorTorque(double torque) {

}

void spAWSDCar::ApplyTransmissionDifferentialCoupling() {

}
