#include <spirit/Objects/spBox.h>

spBox::spBox(const spPose& pose, const spBoxSize& size, double mass, const spColor& color, btDiscreteDynamicsWorld* dynamics_world) {
  color_ = color;
  mass_ = mass;
  index_gui_ = -1;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  object_type_ = spObjectType::BOX;
  btCollisionShape* shape = new btBoxShape(btVector3(1.0,1.0,1.0));
  btTransform tr;
  tr.setIdentity();
  rigid_body_ = CreateRigidBody(mass,spPose2btTransform(pose),shape);
  int box_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH | BulletCollissionType::COL_CHASSIS | BulletCollissionType::COL_WHEEL;
  dynamics_world->addRigidBody(rigid_body_,BulletCollissionType::COL_BOX,box_collides_with_);
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);
  obj_guichanged_ = true;
  SetDimensions(size);
  SetMass(mass);
  rigid_body_->setGravity(dynamics_world->getGravity());
  rigid_body_->setFriction(0);
  rigid_body_->setRollingFriction(0);
}

spBox::~spBox() {
}

void spBox::SetFriction(double fric_coeff) {
  rigid_body_->setFriction(fric_coeff);
}

double spBox::GetFriction() {
  return rigid_body_->getFriction();
}

void spBox::SetRollingFriction(double fric_coeff) {
  rigid_body_->setRollingFriction(fric_coeff);
}

double spBox::GetRollingFriction() {
  return rigid_body_->getRollingFriction();
}

void spBox::SetDimensions(const spBoxSize& dims) {
  // reset box size
  rigid_body_->getCollisionShape()->setLocalScaling(btVector3(dims[0]/2.0,dims[1]/2.0,dims[2]/2.0)*WSCALE);
  obj_guichanged_ = true;
}

spBoxSize spBox::GetDimensions() {
  btVector3 btdims = rigid_body_->getCollisionShape()->getLocalScaling();
  return spBoxSize(2*btdims[0],2*btdims[1],2*btdims[2])*WSCALE_INV;
}

void spBox::SetPose(const spPose& pose) {
  rigid_body_->setWorldTransform(spPose2btTransform(pose));
  obj_guichanged_ = true;
}

const spPose& spBox::GetPose(){
  return btTransform2spPose(rigid_body_->getWorldTransform());
}

void spBox::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spBox::GetColor() {
  return color_;
}


void spBox::SetMass(double mass) {
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
  rigid_body_->getCollisionShape()->calculateLocalInertia(mass,localInertia);
  // reset object mass
  rigid_body_->setMassProps(mass,localInertia);
  mass_ = mass;
}

double spBox::GetMass() {
  return mass_;
}

void spBox::ClampToSurface() {
  obj_clamptosurface_ = true;
}
