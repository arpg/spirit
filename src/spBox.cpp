#include <spirit/Objects/spBox.h>

spBox::spBox(const spPose& pose, const spBoxSize& size, double mass, const spColor& color, btDiscreteDynamicsWorld* dyn_world, btAlignedObjectArray<btCollisionShape*>& col_shapes) {
  color_ = color;
  index_gui_ = -1;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  object_type_ = spObjectType::BOX;
  dynamics_world_ = dyn_world;
  btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
  col_shapes.push_back(shape);
  rigid_body_ = this->CreateRigidBody(mass,spPose2btTransform(pose),shape);
  this->SetMass(mass_);
  this->SetDimensions(size);
  int box_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH | BulletCollissionType::COL_CHASSIS | BulletCollissionType::COL_WHEEL;
  dynamics_world_->addRigidBody(rigid_body_,BulletCollissionType::COL_BOX,box_collides_with_);
  rigid_body_->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  std::cout << "user index is " << rigid_body_->getUserIndex() << std::endl;
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);
  rigid_body_->setGravity(dynamics_world_->getGravity());
  obj_guichanged_ = true;
}

spBox::~spBox() {
  delete(rigid_body_);
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
  rigid_body_->getCollisionShape()->setLocalScaling(btVector3(dims[0]/2,dims[1]/2,dims[2]/2)*WSCALE);
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
  btCollisionObject* obj = dynamics_world_->getCollisionObjectArray()[0];
  std::cout << "origin is " << obj->getWorldTransform().getOrigin()[2]*WSCALE_INV << std::endl;
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
  if (this->IsDynamic()) {
      // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
      rigid_body_->getCollisionShape()->calculateLocalInertia(mass,localInertia);
  }
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
