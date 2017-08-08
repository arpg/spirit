#include <spirit/Objects/spAWSDCar.h>

spAWSDCar::spAWSDCar(const spVehicleConstructionInfo& vehicle_info, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world):spVehicle(vehicle_info,dynamics_world) {
  // check if object has been initialized with four wheels
  if(vehicle_info.wheels_anchor.size() != 4) {
    SPERROREXIT("AWSDCar should have four wheels.");
  }
  // Initialize vehicle with some parameters
  for(int ii=0;ii<4;ii++)
  {
    GetWheel(ii)->EnableSteeringServo(true);
    GetWheel(ii)->SetSteeringServoMaxVelocity(0);
    GetWheel(ii)->SetSteeringServoTorque(10);
    GetWheel(ii)->SetSteeringServoTargetAngle(0);
    GetWheel(ii)->EnableDriveMotor(true);
    GetWheel(ii)->SetDriveMotorTargetVelocity(0);
    GetWheel(ii)->SetDriveMotorTorque(10);
  }
}

spAWSDCar::~spAWSDCar(){
}

void spAWSDCar::SetFrontSteeringAngle(double angle) {
  GetWheel(0)->SetSteeringServoTargetAngle(angle);
  GetWheel(3)->SetSteeringServoTargetAngle(angle);
}

void spAWSDCar::SetRearSteeringAngle(double angle) {
  GetWheel(1)->SetSteeringServoTargetAngle(angle);
  GetWheel(2)->SetSteeringServoTargetAngle(angle);
}

void spAWSDCar::SetEngineMaxVel(double vel) {
  for(int ii=0;ii<4;ii++) {
    GetWheel(ii)->SetDriveMotorTargetVelocity(vel);
  }
}

void spAWSDCar::SetEngineTorque(double torque) {
  // bullet doesn't like zero torque
  if(torque == 0) {
    torque = 1e-10;
  }
  for(int ii=0;ii<4;ii++) {
    GetWheel(ii)->SetDriveMotorTorque(torque);
  }
}

void spAWSDCar::SetSteeringServoMaxVel(double vel){
  for(int ii=0;ii<4;ii++) {
    GetWheel(ii)->SetSteeringServoMaxVelocity(vel);
  }
}

void spAWSDCar::SetSteeringServoTorque(double torque){
  for(int ii=0;ii<4;ii++) {
    GetWheel(ii)->SetSteeringServoTorque(torque);
  }
}

void spAWSDCar::ApplyTransmissionDifferentialCoupling() {

}
