#include <spirit/Objects/spBike.h>

spBike::spBike(const spVehicleConstructionInfo& vehicle_info, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world):spVehicle(vehicle_info,dynamics_world) {
  // check if object has been initialized with four wheels
  if(vehicle_info.wheels_anchor.size() != 2) {
    SPERROREXIT("Bike should have two wheels.");
  }
  // Initialize vehicle with some parameters

  GetWheel(0)->EnableSteeringServo(true);
  GetWheel(0)->SetSteeringServoTargetAngle(0);
  GetWheel(1)->EnableDriveMotor(true);
  GetWheel(1)->SetDriveMotorTargetVelocity(0);

}

spBike::~spBike(){
}

void spBike::SetFrontSteeringAngle(double angle) {
  GetWheel(0)->SetSteeringServoTargetAngle(angle);
}

void spBike::SetRearSteeringAngle(void) {
  GetWheel(1)->SetSteeringServoTargetAngle(0);
}

void spBike::SetEngineMaxVel(double vel) {
   GetWheel(1)->SetDriveMotorTargetVelocity(vel);

}

void spBike::SetEngineTorque(double torque) {
   GetWheel(1)->SetDriveMotorTorque(torque);

}

/*
void spBike::UpdateWheelFriction(double friction) {
    for(int ii=0;ii<4;ii++) {
    GetWheel(ii)->SetFriction(friction);
  }
}
*/

//void spBike::SetSteeringServoMaxVel(double vel){
//  for(int ii=0;ii<4;ii++) {
//    GetWheel(ii)->SetSteeringServoMaxVelocity(vel);
//  }
//}

//void spBike::SetSteeringServoTorque(double torque){
//  for(int ii=0;ii<4;ii++) {
//    GetWheel(ii)->SetSteeringServoTorque(torque);
//  }
//}
