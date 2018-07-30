#include <spirit/Objects/spWheel.h>


spWheel::spWheel(const spVehicleConstructionInfo& vehicle_info, int wheel_index, std::shared_ptr<btRigidBody> chassis_body, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world) {
  color_ = spColor(1,1,1);
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::WHEEL;
  mass_ = vehicle_info.wheel_mass;
  radius_ = vehicle_info.wheel_radius;
  width_ = vehicle_info.wheel_width;

  // calculate wheel origin in world
  chassis_anchor = vehicle_info.wheels_anchor[wheel_index];
  if(dynamics_world && chassis_body){
    phy_engine_ = true;
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(chassis_anchor[0],chassis_anchor[1],chassis_anchor[2]-vehicle_info.susp_preloading_spacer)*WSCALE);
    wheel_shape_ = std::make_shared<btCapsuleShapeX>((vehicle_info.wheel_radius)*WSCALE,(vehicle_info.wheel_width/2.0)*WSCALE);
    //  wheel_shape_ = std::make_shared<btCylinderShapeX>(btVector3(vehicle_info.wheel_width/2,vehicle_info.wheel_radius,vehicle_info.wheel_radius)*WSCALE);
    //  mass_ = vehicle_info.wheel_mass;
    CreateRigidBody(vehicle_info.wheel_mass,tr,wheel_shape_);
    std::shared_ptr<btRigidBody> bodyB = rigid_body_;
    bodyB->setDamping(0,0);
    bodyB->setAngularVelocity(btVector3(0,0,0));
    bodyB->setLinearVelocity(btVector3(0,0,0));
    int wheel_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
    dynamics_world->addRigidBody(bodyB.get(),BulletCollissionType::COL_WHEEL,wheel_collides_with_);
    bodyB->setRollingFriction(vehicle_info.wheel_rollingfriction);
    bodyB->setFriction(vehicle_info.wheel_friction);
    bodyB->setActivationState(DISABLE_DEACTIVATION);
    btVector3 parent_axis(0,0,1);
    btVector3 child_axis(1,0,0);
    btVector3 anchor = tr.getOrigin();
    //  hinge_ = new btHinge2Constraint(*chassis_body,*bodyB,anchor, parent_axis, child_axis);
    hinge_ = std::make_shared<btHinge2Constraint>(*chassis_body.get(),*bodyB.get(),anchor, parent_axis, child_axis);
    // set suspension damping to axis 2 of constraint only (z direction)
    hinge_->setDamping(2,vehicle_info.susp_damping);
    hinge_->setStiffness(2,vehicle_info.susp_stiffness);
    // fix x,y linear movement directions and only move in z direction
    hinge_->setLinearLowerLimit(btVector3(0,0,vehicle_info.susp_preloading_spacer+vehicle_info.susp_lower_limit)*WSCALE);
    hinge_->setLinearUpperLimit(btVector3(0,0,vehicle_info.susp_preloading_spacer+vehicle_info.susp_upper_limit)*WSCALE);
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
    hinge_->setAngularLowerLimit(btVector3(1,0,vehicle_info.steering_servo_lower_limit));
    hinge_->setAngularUpperLimit(btVector3(-1,0,vehicle_info.steering_servo_upper_limit));
    // set motor torque
    if(vehicle_info.engine_torque < MIN_ENGINE_TORQUE) {
        hinge_->setMaxMotorForce(drive_motor_axis,MIN_ENGINE_TORQUE*WSCALE*WSCALE);
    } else {
        hinge_->setMaxMotorForce(drive_motor_axis,vehicle_info.engine_torque*WSCALE*WSCALE);
    }
    // set steering servo torque and max velocity
    hinge_->setMaxMotorForce(steering_servo_axis,vehicle_info.steering_servo_torque*WSCALE*WSCALE);
    hinge_->setTargetVelocity(steering_servo_axis,vehicle_info.steering_servo_max_velocity);

    // add the hinge constraint to the world and disable collision between bodyA/bodyB
    dynamics_world->addConstraint(hinge_.get(),true);
  }
  else{
     phy_engine_ = false;
  }
}

spWheel::~spWheel()
{
//  delete hinge_->getRigidBodyB().getCollisionShape();
//  delete hinge_;
}

void spWheel::SetFriction(double friction) {
    if(phy_engine_){
      rigid_body_->setFriction(friction);
    }
    else{
      friction_ = friction;
    }
}

void spWheel::SetPose(const spPose& pose)
{
  if(phy_engine_){
    btTransform tr;
    spPose2btTransform(pose,tr);
    rigid_body_->setWorldTransform(tr);
  }
  else{
    pose_ = pose;
  }
  obj_guichanged_ = true;
}

const spPose& spWheel::GetPose()
{
  if(phy_engine_){
    btTransform2spPose(rigid_body_->getWorldTransform(),pose_);
  }
  return pose_;
}

void spWheel::SetColor(const spColor& color)
{
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spWheel::GetColor() {
  return color_;
}

double spWheel::GetRadius()
{
  return radius_;
}

double spWheel::GetWidth()
{
  return width_;
}

void spWheel::SetSteeringServoLowerLimit(double limit)
{
  if(phy_engine_){
    hinge_->setAngularLowerLimit(btVector3(1,0,limit));
  }
  else{
    steering_lower_limit_ = limit;
  }
}

void spWheel::SetSteeringServoUpperLimit(double limit)
{
  if(phy_engine_){
    hinge_->setAngularUpperLimit(btVector3(-1,0,limit));
  }
  else{
    steering_upper_limit_ = limit;
  }
}

void spWheel::SetSteeringServoMaxVelocity(double velocity)
{
  if(phy_engine_){
    hinge_->setTargetVelocity(steering_servo_axis,velocity);
  }
  else{
    steering_max_velocity_ = velocity;
  }
}

void spWheel::SetSteeringServoTorque(double torque)
{
//  steering_servo_torque_ = torque;
  if(phy_engine_){
    hinge_->setMaxMotorForce(steering_servo_axis,torque*WSCALE*WSCALE);
  }
  else{
    steering_torque_ = torque;
  }
}

void spWheel::EnableDriveMotor(bool status)
{
//  has_drive_motor_ = status;
  if(phy_engine_){
    hinge_->enableMotor(drive_motor_axis,status);
  }
  else{
    drive_motor_ = status;
  }

}

void spWheel::EnableSteeringServo(bool status)
{
//  has_steering_servo_ = status;
  if(phy_engine_){
    hinge_->enableMotor(steering_servo_axis,status);
    hinge_->setServo(steering_servo_axis,status);
  }
  else{
    steering_servo_ = status;
  }
}

void spWheel::SetDriveMotorTargetVelocity(double velocity)
{
//  drive_motor_target_velocity_ = velocity;
  if(phy_engine_){
    hinge_->setTargetVelocity(drive_motor_axis,velocity);
  }
  else{
    drive_motor_target_velocity_ = velocity;
  }
}

void spWheel::SetDriveMotorTorque(double torque)
{
  if(phy_engine_){
    hinge_->setMaxMotorForce(drive_motor_axis,torque*WSCALE*WSCALE);
  }
  else{
    drive_motor_torque_ = torque;
  }
}

const spTranslation& spWheel::GetChassisAnchor()
{
  return chassis_anchor;
}

double spWheel::GetSteeringServoCurrentAngle()
{
  if(phy_engine_){
    return hinge_->getAngle1();
  }
  else{
    SPERROR("Can only return servo angle when using bullet");
    return 0;
  }
}

void spWheel::SetSteeringServoTargetAngle(double angle)
{
  if(phy_engine_){
      btVector3 limit;
      hinge_->getAngularUpperLimit(limit);
      if(angle>limit[2]) {
        angle = limit[2];
      }
      hinge_->getAngularLowerLimit(limit);
      if(angle<limit[2]) {
        angle = limit[2];
      }
      hinge_->setServoTarget(steering_servo_axis,angle);
  }
  else{
      steering_servo_target_angle_ = angle;
  }
}

void spWheel::SetWheelSpeed(double rps) {


  if(phy_engine_){
      btQuaternion global_rotation_inv(rigid_body_->getWorldTransform().getRotation().inverse());
      btVector3 global_vel(rigid_body_->getAngularVelocity());
      btVector3 local_vel(global_vel.rotate(global_rotation_inv.getAxis(),global_rotation_inv.getAngle()));
      local_vel[0] = -rps;
      btQuaternion global_rotation(rigid_body_->getWorldTransform().getRotation());
      rigid_body_->setAngularVelocity(local_vel.rotate(global_rotation.getAxis(),global_rotation.getAngle()));
  }
  else{
    wheel_speed_ = rps;
  }
}

double spWheel::GetWheelSpeed() {
  if(phy_engine_){
      btQuaternion rot_inv(rigid_body_->getWorldTransform().getRotation().inverse());
      return -(rigid_body_->getAngularVelocity().rotate(rot_inv.getAxis(),rot_inv.getAngle()))[0];
  }
  else{
    return wheel_speed_;
  }
}

const spRotVel& spWheel::GetRotVel(){
  if(phy_engine_){
    rotvel_ = spRotVel(rigid_body_->getAngularVelocity()[0],rigid_body_->getAngularVelocity()[1],rigid_body_->getAngularVelocity()[2]);
  }
  return rotvel_;
}

const spLinVel& spWheel::GetLinVel(){
  if(phy_engine_){
    linvel_ = spLinVel(rigid_body_->getLinearVelocity()[0],rigid_body_->getLinearVelocity()[1],rigid_body_->getLinearVelocity()[2])*WSCALE_INV;
  }
  return linvel_;
}

void spWheel::SetLinVel(const spLinVel& vel) {
  if(phy_engine_){
    rigid_body_->setLinearVelocity(btVector3(vel[0],vel[1],vel[2])*WSCALE);
  }
  else{
    linvel_ = vel;
  }
}

void spWheel::SetAngularVel(const spRotVel& vel) {
  if(phy_engine_){
    rigid_body_->setAngularVelocity(btVector3(vel[0],vel[1],vel[2]));
  }
  else{
    rotvel_ = vel;
  }
}

void spWheel::InitializeSteeringServoAngle(double angle){
    if(phy_engine_){
        double curr_angle = GetSteeringServoCurrentAngle();
        Eigen::AngleAxisd rot(curr_angle-angle,Eigen::Vector3d::UnitZ());
        spPose ps(GetPose());
        ps.rotate(rot);
        SetPose(ps);
        SetSteeringServoTargetAngle(angle);
    }
    else{
        Eigen::AngleAxisd rot(angle,Eigen::Vector3d::UnitZ());
        spPose ps(GetPose());
        ps.rotate(rot);
        SetPose(ps);
        SetSteeringServoTargetAngle(angle);
    }
}




//double spWheel::GetSuspensionLength(){
////  hinge_->calculateTransforms();
//  Eigen::Vector3d susp_len(hinge_->getRelativePivotPosition(0),hinge_->getRelativePivotPosition(1),hinge_->getRelativePivotPosition(2));
//  std::cout << "pose is "
//            << " , " << hinge_->getRelativePivotPosition(2)//susp_len.norm()
//            << std::endl;
//}


//void spWheel::SetSuspensionLength(double length) {

//}
