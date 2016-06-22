#include <spirit/Physics/spBulletWorld.h>

spBulletWorld::spBulletWorld() {
  world_params_.worldMax.setValue(1000,1000,1000);
  world_params_.worldMin.setValue(-1000,-1000,-1000);
  world_params_.solver = spPhysolver::SEQUENTIAL_IMPULSE;
}

spBulletWorld::~spBulletWorld() {
	//delete collision shapes
	for (int j=0;j<collisionShapes_.size();j++)
	{
		btCollisionShape* shape = collisionShapes_[j];
		delete shape;
	}

  delete(dynamics_world_);
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZING:
      delete(solver_dantzig_);
      break;
    case spPhysolver::MLCP_PROJECTEDGAUSSSEIDEL:
      delete(solver_gseidel_);
      break;
    case spPhysolver::SEQUENTIAL_IMPULSE:
      // do nothing
      break;
  }
  delete(solver_);
  delete(broadphase_);
  delete(dispatcher_);
  delete(collisionConfiguration_);
}

bool spBulletWorld::InitEmptyDynamicsWorld() {

  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
  broadphase_ = new btAxisSweep3(world_params_.worldMin,world_params_.worldMax);
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZING:
      solver_dantzig_ = new btDantzigSolver();
      solver_ = new btMLCPSolver(solver_dantzig_);
      break;
    case spPhysolver::MLCP_PROJECTEDGAUSSSEIDEL:
      solver_gseidel_ = new btSolveProjectedGaussSeidel;
      solver_ = new btMLCPSolver(solver_gseidel_);
      break;
    case spPhysolver::SEQUENTIAL_IMPULSE:
      solver_ = new btSequentialImpulseConstraintSolver();
      break;
  }

  dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_,broadphase_,solver_,collisionConfiguration_);

  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZING:
      //for direct solver it is better to have a small A matrix
      dynamics_world_->getSolverInfo().m_minimumSolverBatchSize = 1;
      break;
    case spPhysolver::MLCP_PROJECTEDGAUSSSEIDEL:
      //for direct solver it is better to have a small A matrix
      dynamics_world_->getSolverInfo().m_minimumSolverBatchSize = 1;
      break;
    case spPhysolver::SEQUENTIAL_IMPULSE:
      //for direct solver, it is better to solve multiple objects together, small batches have high overhead
      dynamics_world_->getSolverInfo().m_minimumSolverBatchSize = 128;
      break;
  }

  dynamics_world_->setGravity(btVector3(0,0,-9.80665));
  dynamics_world_->getSolverInfo().m_numIterations = 100;
  return true;
}

btRigidBody* spBulletWorld::CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape) {
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
//	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	return body;
}

void spBulletWorld::AddNewPhyObject(spCommonObject &sp_obj) {
  switch (sp_obj.GetObjecType()) {
    case spObjectType::BOX :
    {
      btRigidBody* thebox = CreateBulletBoxObject((spBox&)sp_obj);
      UpdateBulletBoxObject((spBox&)sp_obj,thebox);
      break;
    }
    case spObjectType::CAR :
    {
      btRigidBody* thecar = CreateBulletCarObject((spCar&)sp_obj);
      UpdateBulletCarObject((spCar&)sp_obj,thecar);
      break;
    }
  }
}

btRigidBody* spBulletWorld::CreateBulletCarObject(spCar& source_obj) {
  // Here we are gonna create a bullet car from scratch
  // Create a bullet compound shape
  btCompoundShape* compound = new btCompoundShape();
  collisionShapes_.push_back(compound);
  // add chassis as a box to compound shape
  // trick: to be able to dynamically change dimentions of shapes initialize them with 1 and scale them later. if we don't do this we need to delete and add shapes each time there is a size change
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(1,1,1));
  collisionShapes_.push_back(chassis_shape);
  //cg_tr effectively shifts the center of mass with respect to the chassis
  // this transform is to put the cog in the right spot
//  spPose chassis_transform(spPose::Identity());
//  spPose cog_transform(spPose::Identity());
//  cog_transform.translate(source_obj.GetLocalCOG());
//  chassis_transform = source_obj.GetPose() * cog_transform.inverse();
//  compound->addChildShape(spPose2btTransform(chassis_transform),chassis_shape);
#warning "we are not doing the cog transform for now, but it should be fixed later"
  compound->addChildShape(spPose2btTransform(spPose::Identity()),chassis_shape);  // test
  // create a rigidbody from compound shape and add it world

  // apply cog transform
//  spPose global_cog(source_obj.GetPose() * cog_transform);
//  btRigidBody* bodyA = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(global_cog),compound);
  btRigidBody* bodyA = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(source_obj.GetPose()),compound);  // test

  dynamics_world_->addRigidBody(bodyA);
  // set the correct index for spCar object so we can access this object later
  bodyA->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  source_obj.SetPhyIndex(bodyA->getUserIndex());
  // now create and add wheels
  for(int ii=0 ; ii<source_obj.GetNumberOfWheels() ; ii++) {
    spWheel* spwheel = source_obj.GetWheel(ii);
    bodyA->setActivationState(DISABLE_DEACTIVATION);
    btTransform tr;
    tr.setIdentity();
#warning "wheel origin is different than wheel pose, make sure it has been implemented correctly"
    tr.setOrigin(btVector3(source_obj.GetWheelOrigin(ii)[0], source_obj.GetWheelOrigin(ii)[1], source_obj.GetWheelOrigin(ii)[2]));
    btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(1,1,1));
    btRigidBody* bodyB = CreateRigidBody(spwheel->GetMass(),tr,wheel_shape);
    dynamics_world_->addRigidBody(bodyB);
    bodyB->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
    spwheel->SetPhyIndex(bodyB->getUserIndex());
    bodyB->setFriction(spwheel->GetFriction());
    bodyB->setActivationState(DISABLE_DEACTIVATION);
    btVector3 parent_axis(0,1,0);
    btVector3 child_axis(1,0,0);
    btVector3 anchor = tr.getOrigin();
    btHinge2Constraint* hinge = new btHinge2Constraint(*bodyA,*bodyB,anchor, parent_axis, child_axis);
    // set suspension damping to axis 2 of constraint only (z direction)
    hinge->setDamping(2,spwheel->GetSuspDamping());
    // fix x,y linear movement directions and only move in z direction
    hinge->setLinearLowerLimit(btVector3(0,0,spwheel->GetSuspLowerLimit()));
    hinge->setLinearUpperLimit(btVector3(0,0,spwheel->GetSuspUpperLimit()));
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
    hinge->setAngularLowerLimit(btVector3(1,0,spwheel->GetSteeringLowerLimit()));
    hinge->setAngularUpperLimit(btVector3(-1,0,spwheel->GetSteeringLowerLimit()));
    // add motors if required
    int drive_motor_axis = 3;
    int steering_motor_axis = 5;
    if(spwheel->GetHasDriveMotor()) {
      hinge->enableMotor(drive_motor_axis,true);
    }
    if(spwheel->GetHasSteeringMotor()) {
      hinge->enableMotor(steering_motor_axis,true);
    }
    // add the hinge constraint to the world and disable collision between bodyA/bodyB
    dynamics_world_->addConstraint(hinge,true);
  }
  return bodyA;
}


void spBulletWorld::UpdateBulletCarObject(spCar& source_obj, btRigidBody* dest_obj) {
  // get compoundshape from rigidbody
  btCompoundShape* compound = (btCompoundShape*) dest_obj->getCollisionShape();
  // since we only added chassis(box) to compound shape its gonna be in index_0 shape
  int box_childnumber = 0;
  btCollisionShape* chassis_shape = compound->getChildShape(box_childnumber);
  // Here we should update all properties of the car
  // reset box size
  spBoxSize dims(source_obj.GetChassisSize());
  chassis_shape->setLocalScaling(btVector3(dims[0]/2,dims[1]/2,dims[2]/2));

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
  chassis_shape->calculateLocalInertia(source_obj.GetChassisMass(),localInertia);
  // reset COG from spObject
  dest_obj->setMassProps(source_obj.GetChassisMass(),localInertia);
//  spPose chassis_transform(spPose::Identity());
//  spPose cog_transform(spPose::Identity());
//  cog_transform.translate(source_obj.GetLocalCOG());
//  chassis_transform = source_obj.GetPose() * cog_transform.inverse();
//  compound->updateChildTransform(box_childnumber,spPose2btTransform(chassis_transform));
//  spPose global_cog(source_obj.GetPose() * cog_transform);
#warning "we are not doing the cog transform for now, but it should be fixed later"
  spPose global_cog(source_obj.GetPose());
  dest_obj->setWorldTransform(spPose2btTransform(global_cog));
  // Update wheel
  for(int ii=0; ii<source_obj.GetNumberOfWheels(); ii++){
    spWheel* spwheel = source_obj.GetWheel(ii);
    btRigidBody* wheel_body = &dest_obj->getConstraintRef(ii)->getRigidBodyB();
    // resize wheel
    btVector3 wheel_dim(spwheel->GetWidth()/2,spwheel->GetRadius()/2,spwheel->GetRadius()/2);
    wheel_body->getCollisionShape()->setLocalScaling(wheel_dim);
    // calculate and set inertia/mass
    btVector3 wheel_local_inertia(0,0,0);
    double wheel_mass = spwheel->GetMass();
    wheel_body->getCollisionShape()->calculateLocalInertia(wheel_mass,wheel_local_inertia);
    wheel_body->setMassProps(wheel_mass,wheel_local_inertia);
    // reset wheel location
#warning "removed updating wheel origin, I guess this is something to be decided only by phy engine"
//    btTransform wheel_tr;
//    wheel_tr.setIdentity();
//    wheel_tr.setOrigin(btVector3(source_obj.GetWheelOrigin(ii)[0], source_obj.GetWheelOrigin(ii)[1], source_obj.GetWheelOrigin(ii)[2]));
//    wheel_body->setWorldTransform(wheel_tr);
    // reset wheel friction and damping
    wheel_body->setFriction(spwheel->GetFriction());

    // reset suspension damping to axis 2 of constraint only (z direction)
    btHinge2Constraint* hinge = (btHinge2Constraint*) dest_obj->getConstraintRef(ii);
    hinge->setDamping(2,spwheel->GetSuspDamping());
    // fix x,y linear movement directions and only move in z direction
    hinge->setLinearLowerLimit(btVector3(0,0,spwheel->GetSuspLowerLimit()));
    hinge->setLinearUpperLimit(btVector3(0,0,spwheel->GetSuspUpperLimit()));
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction
    hinge->setAngularLowerLimit(btVector3(1,0,spwheel->GetSteeringLowerLimit()));
    hinge->setAngularUpperLimit(btVector3(-1,0,spwheel->GetSteeringLowerLimit()));
    // add motors if required
    int drive_motor_axis = 3;
    int steering_motor_axis = 5;
    if(spwheel->GetHasDriveMotor()) {
      hinge->setTargetVelocity(drive_motor_axis,spwheel->GetDriveMotorTargetVelocity());
      hinge->setMaxMotorForce(drive_motor_axis,spwheel->GetDriveMotorTorque());
    }
    if(spwheel->GetHasSteeringMotor()) {
      hinge->setTargetVelocity(steering_motor_axis,spwheel->GetSteeringMotorTargetVelocity());
      hinge->setMaxMotorForce(steering_motor_axis,spwheel->GetSteeringMotorTorque());
    }
  }
}
#warning "returning bttransform might not be the fastest way of type conversion"
btTransform spBulletWorld::spPose2btTransform(const spPose& pose) {
  btTransform tr;
  tr.setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2]));
  spRotation q(pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  return tr;
}
#warning "if I return a reference instead of sppose UpdateSpiritObjectsFromPhy() will mess up things"
spPose spBulletWorld::btTransform2spPose(const btTransform& tr) {
  spPose pose(spPose::Identity());
  btVector3 origin = tr.getOrigin();
  pose.translate(spTranslation(origin[0],origin[1],origin[2]));
  btQuaternion btrot = tr.getRotation();
  spRotation spangle(btrot.w(),btrot.x(),btrot.y(),btrot.z());
  pose.rotate(spangle);
  return pose;
}


btRigidBody* spBulletWorld::CreateBulletBoxObject(spBox &source_obj) {
  btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
  collisionShapes_.push_back(shape);
  btTransform tr;
  tr.setIdentity();
  btRigidBody* dest_obj = CreateRigidBody(1,tr,shape);
  dynamics_world_->addRigidBody(dest_obj);
  dest_obj->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  source_obj.SetPhyIndex(dest_obj->getUserIndex());
  return dest_obj;
}


// update all parameters of a box
void spBulletWorld::UpdateBulletBoxObject(spBox &source_obj, btRigidBody *dest_obj) {
  // reset box size
  spBoxSize dims(source_obj.GetDimensions());
  dest_obj->getCollisionShape()->setLocalScaling(btVector3(dims[0]/2,dims[1]/2,dims[2]/2));
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  btVector3 localInertia(0,0,0);
  if (source_obj.IsDynamic()) {
      // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
      dest_obj->getCollisionShape()->calculateLocalInertia(source_obj.GetMass(),localInertia);
  }
  // reset object mass
  dest_obj->setMassProps(source_obj.GetMass(),localInertia);
  // transform phy object
//  std::cout << "source_obj pose" << btTransform2spPose(dest_obj->getWorldTransform()).matrix() << std::endl;
  dest_obj->setWorldTransform(spPose2btTransform(source_obj.GetPose()));
}


void spBulletWorld::UpdatePhyObjectsFromSpirit(Objects &spobj) {
  // go through all spirit objects
  for(int ii=0; ii<spobj.GetNumOfObjects(); ii++) {
    //only update objects which had physics property changes
    if(spobj.GetObject(ii).HasChangedPhy()) {
      // get gui index of object
      int phy_index = spobj.GetObject(ii).GetPhyIndex();
      // update the phy object
      switch (spobj.GetObject(ii).GetObjecType()) {
        case spObjectType::BOX:
        {
          btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
          btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
          UpdateBulletBoxObject((spBox&)spobj.GetObject(ii),bulletbody);
          spobj.GetObject(ii).SetPhyUpdated();
          break;
        }
        case spObjectType::CAR:
        {
          btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
          btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
          UpdateBulletCarObject((spCar&)spobj.GetObject(ii),bulletbody);
          spobj.GetObject(ii).SetPhyUpdated();
          break;
        }
      }
    }
  }
}

void spBulletWorld::UpdateSpiritObjectsFromPhy(Objects &spobjects) {
  for(int ii=0; ii<spobjects.GetNumOfObjects(); ii++) {
    //only update objects which are dynamic
    if(spobjects.GetObject(ii).IsDynamic()) {
      switch (spobjects.GetObject(ii).GetObjecType()) {
        case spObjectType::BOX:
        {
          spBox& box = (spBox&) spobjects.GetObject(ii);
          // update the phy object
          btCollisionObject* obj = dynamics_world_->getCollisionObjectArray()[box.GetPhyIndex()];
          spPose ps(btTransform2spPose(obj->getWorldTransform()));
          box.SetPose(ps);
          break;
        }
        case spObjectType::CAR:
        {
#error "car wheels are not oriented correctly, it has a initialization issue, in first frame all wheels are in one point of space"
          spCar& car = (spCar&) spobjects.GetObject(ii);
          // update chassis
          btCollisionObject* chassis_obj = dynamics_world_->getCollisionObjectArray()[car.GetPhyIndex()];
          car.SetPose(btTransform2spPose(chassis_obj->getWorldTransform()));
          // update wheels
          for(int ii=0; ii<car.GetNumberOfWheels(); ii++) {
            btCollisionObject* wheel_obj = dynamics_world_->getCollisionObjectArray()[car.GetWheel(ii)->GetPhyIndex()];
            car.GetWheel(ii)->SetPose(btTransform2spPose(wheel_obj->getWorldTransform()));
          }
          break;
        }
      }
    }
  }
}

void spBulletWorld::StepPhySimulation(double step_time) {
#warning "what is 10 here ..."
  dynamics_world_->stepSimulation(step_time,10);
}







