#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(const spVehicleConstructionInfo& vehicle_info,btDiscreteDynamicsWorld* dyn_world, btAlignedObjectArray<btCollisionShape*>& col_shapes) {
  dynamics_world_ = dyn_world;
  mass_ = vehicle_info.chassis_mass;
  // Create a bullet compound shape
  btCompoundShape* compound = new btCompoundShape();
  col_shapes.push_back(compound);
  // add chassis as a box to compound shape
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(vehicle_info.chassis_size[0]/2,vehicle_info.chassis_size[1]/2,vehicle_info.chassis_size[2]/2)*WSCALE);
  col_shapes.push_back(chassis_shape);
  // this transform is to put the cog in the right spot
  cog_local_ = spPose::Identity();
  cog_local_.translation() = vehicle_info.cog;
  compound->addChildShape(spPose2btTransform(cog_local_.inverse()),chassis_shape);
  // create a rigidbody from compound shape and add it to world
  btRigidBody* bodyA = CreateRigidBody(vehicle_info.chassis_mass,spPose2btTransform(cog_local_),compound);
  bodyA->setFriction(vehicle_info.chassis_friction);
  int chassis_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
  dynamics_world_->addRigidBody(bodyA,BulletCollissionType::COL_CHASSIS,chassis_collides_with_);
  // set body velocities to zero
  bodyA->setLinearVelocity(btVector3(0,0,0));
  bodyA->setAngularVelocity(btVector3(0,0,0));
  // set damping to zero since we are moving in air
  bodyA->setDamping(0,0);
  bodyA->setActivationState(DISABLE_DEACTIVATION);

  rigid_body_ = bodyA;
//  mass_ = vehicle_info.chassis_mass;
//  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
//    wheel_.push_back(std::make_shared<spWheel>(vehicle_info));
//    wheel_[ii]->SetChassisAnchor(vehicle_info.wheels_anchor[ii]);
//  }

//  pose_ = vehicle_info.pose;
//  MoveWheelsToAnchors();
//  color_ = vehicle_info.color;
//  cog_local_ = spPose::Identity();
//  cog_local_.translation() = vehicle_info.cog;
//  chassis_size_ = vehicle_info.chassis_size;
//  index_phy_ = -1;
//  index_gui_ = -1;
//  obj_phychanged_ = false;
//  obj_guichanged_ = false;
//  modifiable_gui_ = false;
//  obj_clamptosurface_ = false;
//  object_type_ = spObjectType::VEHICLE;
  index_phy_ = -1;
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  rot_vel = spRotVel(0,0,0);
  lin_vel = spLinVel(0,0,0);
  // now create and add wheels
  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>(vehicle_info,ii,rigid_body_,dynamics_world_,col_shapes));
  }
  SetPose(vehicle_info.pose);
  MoveWheelsToAnchors(vehicle_info.pose);
  SetColor(vehicle_info.color);
  chassis_size_ = vehicle_info.chassis_size;
  object_type_ = vehicle_info.vehicle_type;
}

spVehicle::~spVehicle() {}

void spVehicle::SetPose(const spPose& pose) {
  rigid_body_->setWorldTransform(spPose2btTransform(pose));

  // set chassis pose
//  pose_ = pose;
  statevec_.head(3) = pose.translation();
  spRotation quat(pose.rotation());
  statevec_.segment(3,4) << quat.w(),quat.x(),quat.y(),quat.z();
  MoveWheelsToAnchors(pose);
//  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

void spVehicle::MoveWheelsToAnchors(const spPose& chasis_pose) {
  for(int ii=0; ii<wheel_.size(); ii++) {
    spPose sp(spPose::Identity());
    spPose tr = chasis_pose;
    sp.translate(chasis_pose*(wheel_[ii]->GetChassisAnchor()));
//    sp.translate(pose_*(wheel_[ii]->GetChassisAnchor()+spTranslation(wheel_[ii]->GetWidth()/2,0,0)));
//    Eigen::AngleAxisd ang1(-SP_PI/2,Eigen::Vector3d::UnitY());
//    tr.rotate(ang1);
    sp.rotate(tr.rotation());
//    sp.rotate(pose_.rotation());
    wheel_[ii]->SetPose(sp);
  }
}

const spPose& spVehicle::GetPose() { return pose_; }

void spVehicle::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spVehicle::GetColor() {
  return color_;
}

const spPose& spVehicle::GetWheelOrigin(int index)
{
  return wheel_[index]->GetPose();
}

double spVehicle::GetChassisMass() {
  return mass_;
}

void spVehicle::SetChassisMass(double mass) {
  mass_ = mass;
  obj_phychanged_ = true;
}


const spBoxSize& spVehicle::GetChassisSize() {
  return chassis_size_;
}

const spPose& spVehicle::GetLocalCOG(){
  return cog_local_;
}

int spVehicle::GetNumberOfWheels()
{
  return wheel_.size();
}

spWheel*spVehicle::GetWheel(int index)
{

  return wheel_[index].get();
}

void spVehicle::SetVelocity(const spVelocity& chassis_vel) {
  statevec_.tail<6>() = chassis_vel;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spStateVec& spVehicle::GetStateVecor() {
  return statevec_;
}

void spVehicle::SetClampToSurfaceFlag() {
  obj_clamptosurface_ = true;
}

const spLinVel& spVehicle::GetLinVel(){
  return lin_vel;
}

void spVehicle::SetLinVel(const spLinVel& vel) {
 lin_vel = vel;
}

const spRotVel& spVehicle::GetRotVel(){
  return rot_vel;
}

void spVehicle::SetRotVel(const spRotVel& vel) {
 rot_vel = vel;
}
