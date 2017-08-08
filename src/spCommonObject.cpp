#include <spirit/Objects/spCommonObject.h>

spCommonObject::spCommonObject() : rigid_body_(nullptr) {}

spCommonObject::~spCommonObject() {
//  if(rigid_body_ != nullptr) {
//    delete(rigid_body_->getMotionState());
//  }
}

void spCommonObject::SetGuiIndex(int index) { index_gui_ = index; }

int spCommonObject::GetGuiIndex() { return index_gui_; }

bool spCommonObject::HasChangedGui() {
//  bool status = obj_guichanged_;
//  obj_guichanged_ = false;
//  return status;
  return true;
}

// tells if object has been modified since last query
spObjectType spCommonObject::GetObjecType() { return object_type_; }

void spCommonObject::SetGuiUpdated()
{
  obj_guichanged_ = false;
}

bool spCommonObject::IsDynamic() {
  if(mass_>0)
    return true;
  else
    return false;
}

bool spCommonObject::IsGuiModifiable() {
  return modifiable_gui_;
}

bool spCommonObject::NeedsClampToSurface() {
  return obj_clamptosurface_;
}

void spCommonObject::SetClamped() {
  obj_clamptosurface_ = false;
}

// CreateRigidBody must be called only once for each object
void spCommonObject::CreateRigidBody(double mass, const spPose& pose, std::shared_ptr<btCollisionShape> shape) {
	btTransform tr;
	spPose2btTransform(pose,tr);
	CreateRigidBody(mass,tr,shape);
}

// CreateRigidBody must be called only once for each object
void spCommonObject::CreateRigidBody(double mass, const btTransform& tr, std::shared_ptr<btCollisionShape> shape) {
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	motion_state_ = std::make_unique<btDefaultMotionState>(tr);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,motion_state_.get(),shape.get(),localInertia);
//	std::shared_ptr<btRigidBody> body = std::make_shared<btRigidBody>(cInfo);
	rigid_body_ = std::make_shared<btRigidBody>(cInfo);
//	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);
//	return body;
}

void spCommonObject::spPose2btTransform(const spPose& pose, btTransform& tr) {
//  std::shared_ptr<btTransform> tr = std::make_shared<btTransform>();
  tr.setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2])*WSCALE);
  spRotation q(pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
//  return tr;
}

void spCommonObject::btTransform2spPose(const btTransform& tr, spPose& pose) {
  btVector3 origin = tr.getOrigin();
  pose.translate(spTranslation(origin[0],origin[1],origin[2])*WSCALE_INV);
  btQuaternion btrot = tr.getRotation();
  spRotation spangle(btrot.w(),btrot.x(),btrot.y(),btrot.z());
  pose.rotate(spangle);
}

std::shared_ptr<btRigidBody> spCommonObject::GetRigidbody() {
  return rigid_body_;
}

