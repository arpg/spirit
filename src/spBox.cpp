#include <spirit/Objects/spBox.h>

spBox::spBox(const spPose& pose, const spBoxSize& size, double mass, const spColor& color, std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world) {
  color_ = color;
  mass_ = mass;
  index_gui_ = -1;
  modifiable_gui_ = false;
  obj_clamptosurface_ = false;
  object_type_ = spObjectType::BOX;
  obj_guichanged_ = true;

  if(dynamics_world) {
    shape_ = std::make_shared<btBoxShape>(btVector3(1.0,1.0,1.0));
    btTransform tr;
    tr.setIdentity();
    CreateRigidBody(mass,pose,shape_);
    int box_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH | BulletCollissionType::COL_CHASSIS | BulletCollissionType::COL_WHEEL;
    dynamics_world->addRigidBody(rigid_body_.get(),BulletCollissionType::COL_BOX,box_collides_with_);
    rigid_body_->setActivationState(DISABLE_DEACTIVATION);
    rigid_body_->setGravity(dynamics_world->getGravity());
    rigid_body_->setFriction(0);
    rigid_body_->setRollingFriction(0);
    phy_engine_ = true;
  }
  else {
    phy_engine_ = false;
    SetPose(pose);

  }
  SetDimensions(size);
  SetMass(mass);
}

spBox::~spBox() {
//  delete rigid_body_->getCollisionShape();
}

void spBox::SetFriction(double fric_coeff) {
  if(phy_engine_){
    rigid_body_->setFriction(fric_coeff);
  }
  else{
     friction_ = fric_coeff;
  }
}

double spBox::GetFriction() {
  if(phy_engine_){
    return rigid_body_->getFriction();
  }
  else{
    return friction_;
  }
}

void spBox::SetRollingFriction(double fric_coeff) {
  if(phy_engine_){
    rigid_body_->setRollingFriction(fric_coeff);
  }
  else{
    rolling_friction_ = fric_coeff;
  }
}

double spBox::GetRollingFriction() {
  if(phy_engine_){
    return rigid_body_->getRollingFriction();
  }
  else{
    return rolling_friction_;
  }
}

void spBox::SetDimensions(const spBoxSize& dims) {
  if(phy_engine_){
    // reset box size
    rigid_body_->getCollisionShape()->setLocalScaling(btVector3(dims[0]/2.0,dims[1]/2.0,dims[2]/2.0)*WSCALE);
  }
  else{
    size_ = dims;
  }
  obj_guichanged_ = true;
}

spBoxSize spBox::GetDimensions() {
  if(phy_engine_){
    btVector3 btdims = rigid_body_->getCollisionShape()->getLocalScaling();
    return spBoxSize(2*btdims[0],2*btdims[1],2*btdims[2])*WSCALE_INV;
  }
  else{
    return size_;
  }
}

void spBox::SetPose(const spPose& pose) {
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

const spPose& spBox::GetPose(){
  if(phy_engine_){
    btTransform2spPose(rigid_body_->getWorldTransform(),pose_);
  }
  return pose_;
}

void spBox::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spBox::GetColor() {
  return color_;
}

void spBox::SetMass(double mass) {
  if(phy_engine_){
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    btVector3 localInertia(0,0,0);
    // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
    rigid_body_->getCollisionShape()->calculateLocalInertia(mass,localInertia);
    // reset object mass
    rigid_body_->setMassProps(mass,localInertia);
  }
  mass_ = mass;
}

double spBox::GetMass() {
  return mass_;
}

void spBox::ClampToSurface() {
  obj_clamptosurface_ = true;
}
