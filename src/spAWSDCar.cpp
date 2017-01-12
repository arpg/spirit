#include <spirit/Objects/spAWSDCar.h>

spAWSDCar::spAWSDCar(const spVehicleConstructionInfo& vehicle_info, btDiscreteDynamicsWorld* dynamics_world):spVehicle(vehicle_info,dynamics_world) {
  // check if object has been initialized with four wheels
  if(vehicle_info.wheels_anchor.size() != 4) {
    SPERROREXIT("AWSDCar should have four wheels.");
  }
  // Initialize vehicle with some parameters
  for(int ii=0;ii<4;ii++)
  {
    spWheel* wheel = GetWheel(ii);
    wheel->EnableSteeringServo(true);
    wheel->SetSteeringServoMaxVelocity(0);
    wheel->SetSteeringServoTorque(10);
    wheel->SetSteeringServoTargetAngle(0);
    wheel->EnableDriveMotor(true);
    wheel->SetDriveMotorTargetVelocity(0);
    wheel->SetDriveMotorTorque(10);
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
  for(int ii=0;ii<4;ii++)
  {
    spWheel* wheel = GetWheel(ii);
    wheel->SetDriveMotorTargetVelocity(vel);
  }
}

void spAWSDCar::SetEngineTorque(double torque) {
  if(torque == 0) {
    torque = 0.000000001;
  }
  for(int ii=0;ii<4;ii++)
  {
    spWheel* wheel = GetWheel(ii);
    wheel->SetDriveMotorTorque(torque);
  }
}

void spAWSDCar::SetSteeringServoMaxVel(double vel){
  for(int ii=0;ii<4;ii++)
  {
    spWheel* wheel = GetWheel(ii);
    wheel->SetSteeringServoMaxVelocity(vel);
  }
}

void spAWSDCar::SetSteeringServoTorque(double torque){
  for(int ii=0;ii<4;ii++)
  {
    spWheel* wheel = GetWheel(ii);
    wheel->SetSteeringServoTorque(torque);
  }
}

void spAWSDCar::ApplyTransmissionDifferentialCoupling() {

}
