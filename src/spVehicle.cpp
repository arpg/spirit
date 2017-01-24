#include <spirit/Objects/spVehicle.h>

spVehicle::spVehicle(const spVehicleConstructionInfo& vehicle_info,btDiscreteDynamicsWorld* dynamics_world) {
  mass_ = vehicle_info.chassis_mass;
  chassis_size_ = vehicle_info.chassis_size;
  // Create a bullet compound shape
  chassis_compound = new btCompoundShape();
  // add chassis as a box to compound shape
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(vehicle_info.chassis_size[0]/2.0,vehicle_info.chassis_size[1]/2.0,vehicle_info.chassis_size[2]/2.0)*WSCALE);
  // this transform is to put the cog in the right spot
  cog_local_ = spPose::Identity();
  cog_local_.translation() = vehicle_info.cog;
  chassis_compound->addChildShape(spPose2btTransform(cog_local_.inverse()),chassis_shape);
  // create a rigidbody from compound shape and add it to world
  rigid_body_ = CreateRigidBody(vehicle_info.chassis_mass,spPose2btTransform(cog_local_),chassis_compound);
  rigid_body_->setFriction(vehicle_info.chassis_friction);
  int chassis_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
  dynamics_world->addRigidBody(rigid_body_,BulletCollissionType::COL_CHASSIS,chassis_collides_with_);
  // set body velocities to zero
  rigid_body_->setLinearVelocity(btVector3(0,0,0));
  rigid_body_->setAngularVelocity(btVector3(0,0,0));
  // set damping to zero since we are moving in air
  rigid_body_->setDamping(0,0);
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);
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
  // now create and add wheels
  for(int ii=0; ii<vehicle_info.wheels_anchor.size(); ii++) {
    wheel_.push_back(std::make_shared<spWheel>(vehicle_info,ii,rigid_body_,dynamics_world));
  }
  SetPose(vehicle_info.pose);
  MoveWheelsToAnchors(vehicle_info.pose);
  SetColor(vehicle_info.color);
  chassis_size_ = vehicle_info.chassis_size;
  object_type_ = vehicle_info.vehicle_type;
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
//  rot_vel = spRotVel(0,0,0);
//  lin_vel = spLinVel(0,0,0);
}

spVehicle::~spVehicle() {}

void spVehicle::SetPose(const spPose& pose) {
  rigid_body_->setWorldTransform(spPose2btTransform(pose*GetLocalCOG()));
//  statevec_.head(3) = pose.translation();
//  spRotation quat(pose.rotation());
//  statevec_.segment(3,4) << quat.w(),quat.x(),quat.y(),quat.z();
  MoveWheelsToAnchors(pose);
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

const spPose& spVehicle::GetPose() {
  std::shared_ptr<spPose> pose = std::make_shared<spPose>(btTransform2spPose(rigid_body_->getWorldTransform())*GetLocalCOG().inverse());
  return *pose.get();
}

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
  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
  chassis_compound->calculateLocalInertia(mass,localInertia);
  rigid_body_->setMassProps(mass,localInertia);
  mass_ = mass;
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

spWheel* spVehicle::GetWheel(int index)
{

  return wheel_[index].get();
}

void spVehicle::SetVelocity(const spVelocity& chassis_vel) {
//  statevec_.tail<6>() = chassis_vel;
//  obj_guichanged_ = true;
  SPERROREXIT("SetVel not implemented !");
}

const spStateVec& spVehicle::GetStateVecor() {
  spPose pose = btTransform2spPose(rigid_body_->getWorldTransform());
  statevec_[0] = pose.translation()[0];
  statevec_[1] = pose.translation()[1];
  Eigen::Vector3d v = pose.rotation().eulerAngles(1,2,3);
  // take the yaw angle, for some reason its on v[1] instead of expected v[2] position
  statevec_[2] = v[1];
  btVector3 lin_vel = rigid_body_->getLinearVelocity();
  statevec_[3] = lin_vel[0];
  statevec_[4] = lin_vel[1];
  btVector3 rot_vel = rigid_body_->getAngularVelocity();
  statevec_[5] = rot_vel[2];
//  spRotation quat(pose.rotation());
//  statevec_.segment(3,4) << quat.w(),quat.x(),quat.y(),quat.z();
  return statevec_;
}

void spVehicle::SetClampToSurfaceFlag() {
  obj_clamptosurface_ = true;
}
