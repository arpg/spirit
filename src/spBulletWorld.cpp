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
      // pass in nullptr to create a new bullet object and add it to world
      int phy_obj_index = UpdateBulletBoxObject((spBox&)sp_obj,nullptr);
      sp_obj.SetPhyIndex(phy_obj_index);
      break;
    }
    case spObjectType::CAR :
    {
      // pass in nullptr to create a new bullet object
      btRigidBody* compound_body = UpdateBulletCarObject((spCar&)sp_obj,nullptr);
      compound_body->setUserIndex(dynamics_world_->getNumCollisionObjects());
      dynamics_world_->addRigidBody(compound_body);
      sp_obj.SetPhyIndex(compound_body->getUserIndex());
      break;
    }
  }
}


btRigidBody* spBulletWorld::UpdateBulletCarObject(spCar& source_obj, btRigidBody* dest_obj) {
  // if bullet pointer has not been defined yet then assign memory and initialize
  if (dest_obj == nullptr) {
    /// Here we are gonna create a bullet car from scratch
    // Create a bullet compound shape
    btCompoundShape* compound = new btCompoundShape();
    collisionShapes_.push_back(compound);
    // add chassis as a box to compound shape
    // trick: to be able to dynamically change dimentions of shapes initialize them with 1 and scale them later. if we don't do this we need to delete and add shapes each time there is a size change
    btCollisionShape* chassis_shape = new btBoxShape(btVector3(1,1,1));
    collisionShapes_.push_back(chassis_shape);
    //cg_tr effectively shifts the center of mass with respect to the chassis
    // this transform is to put the cog in the right spot
    spPose chassis_transform(spPose::Identity());
    spPose cog_transform(spPose::Identity());
    cog_transform.translate(source_obj.GetLocalCOG());
    chassis_transform = source_obj.GetChassisPose() * cog_transform.inverse();
    compound->addChildShape(spPose2btTransform(chassis_transform),chassis_shape);
    // create a rigidbody from compound shape
    spPose global_cog(source_obj.GetChassisPose() * cog_transform);
    dest_obj = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(global_cog),compound);
    btRigidBody* bodyA = dest_obj;
    // now create and add wheels (we assume they are same size and shape)
    for(int ii=0 ; ii<4 ; ii++) {
      bodyA->setActivationState(DISABLE_DEACTIVATION);
      btTransform tr;
      tr.setIdentity();
      tr.setOrigin(btVector3(source_obj.GetWheelOrigin(ii)[0], source_obj.GetWheelOrigin(ii)[1], source_obj.GetWheelOrigin(ii)[2]));
      //      btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(source_obj.GetWheelWidth(ii),source_obj.GetWheelRadius(ii),source_obj.GetWheelRadius(ii)));;
      btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(1,1,1));
      btRigidBody* bodyB = CreateRigidBody(source_obj.GetWheelMass(ii),tr,wheel_shape);
      bodyB->setFriction(source_obj.GetWheelFriction(ii));
      btVector3 parent_axis(0,1,0);
      btVector3 child_axis(1,0,0);
      btVector3 anchor = tr.getOrigin();
      btHinge2Constraint* hinge = new btHinge2Constraint(*bodyA,*bodyB,anchor, parent_axis, child_axis);
      // set suspension damping to axis 2 of constraint only (z direction)
      hinge->setDamping(2,source_obj.GetWheelSuspDamping(ii));
      // fix x,y linear movement directions and only move in z direction
      hinge->setLinearLowerLimit(btVector3(0,0,source_obj.GetWheelSuspLowerLimit(ii)));
      hinge->setLinearUpperLimit(btVector3(0,0,source_obj.GetWheelSuspUpperLimit(ii)));
      // set rotational directions
      // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
      hinge->setAngularLowerLimit(btVector3(1,0,source_obj.GetWheelSteeringLowerLimit(ii)));
      hinge->setAngularUpperLimit(btVector3(-1,0,source_obj.GetWheelSteeringLowerLimit(ii)));
      // add motors if required
      int drive_motor_axis = 3;
      int steering_motor_axis = 5;
      if(source_obj.GetWheelHasDriveMotor(ii)) {
        hinge->enableMotor(drive_motor_axis,true);
      }
      if(source_obj.GetWheelHasSteeringMotor(ii)) {
        hinge->enableMotor(steering_motor_axis,true);
      }
      // add the hinge constraint to the world and disable collision between bodyA/bodyB
      dynamics_world_->addConstraint(hinge,true);
    }
  }

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
  spPose chassis_transform(spPose::Identity());
  spPose cog_transform(spPose::Identity());
  cog_transform.translate(source_obj.GetLocalCOG());
  chassis_transform = source_obj.GetChassisPose() * cog_transform.inverse();
  compound->updateChildTransform(box_childnumber,spPose2btTransform(chassis_transform));
  spPose global_cog(source_obj.GetChassisPose() * cog_transform);
  dest_obj->setWorldTransform(spPose2btTransform(global_cog));
  // Update wheel
  for(int ii=0; ii<4; ii++){
    btRigidBody* wheel_body = &dest_obj->getConstraintRef(ii)->getRigidBodyB();
    // resize wheel
    btVector3 wheel_dim(source_obj.GetWheelWidth(ii)/2,source_obj.GetWheelRadius(ii)/2,source_obj.GetWheelRadius(ii)/2);
    wheel_body->getCollisionShape()->setLocalScaling(wheel_dim);
    // calculate and set inertia/mass
    btVector3 wheel_local_inertia(0,0,0);
    double wheel_mass = source_obj.GetWheelMass(ii);
    wheel_body->getCollisionShape()->calculateLocalInertia(wheel_mass,wheel_local_inertia);
    wheel_body->setMassProps(wheel_mass,wheel_local_inertia);
    // reset wheel location
    btTransform wheel_tr;
    wheel_tr.setIdentity();
    wheel_tr.setOrigin(btVector3(source_obj.GetWheelOrigin(ii)[0], source_obj.GetWheelOrigin(ii)[1], source_obj.GetWheelOrigin(ii)[2]));
    wheel_body->setWorldTransform(wheel_tr);
    // reset wheel friction and damping
    wheel_body->setFriction(source_obj.GetWheelFriction(ii));

    // reset suspension damping to axis 2 of constraint only (z direction)
    btHinge2Constraint* hinge = (btHinge2Constraint*) dest_obj->getConstraintRef(ii);
    hinge->setDamping(2,source_obj.GetWheelSuspDamping(ii));
    // fix x,y linear movement directions and only move in z direction
    hinge->setLinearLowerLimit(btVector3(0,0,source_obj.GetWheelSuspLowerLimit(ii)));
    hinge->setLinearUpperLimit(btVector3(0,0,source_obj.GetWheelSuspUpperLimit(ii)));
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction
    hinge->setAngularLowerLimit(btVector3(1,0,source_obj.GetWheelSteeringLowerLimit(ii)));
    hinge->setAngularUpperLimit(btVector3(-1,0,source_obj.GetWheelSteeringLowerLimit(ii)));
    // add motors if required
    int drive_motor_axis = 3;
    int steering_motor_axis = 5;
    if(source_obj.GetWheelHasDriveMotor(ii)) {
      hinge->setTargetVelocity(drive_motor_axis,source_obj.GetWheelDriveMotorTargetVelocity(ii));
      hinge->setMaxMotorForce(drive_motor_axis,source_obj.GetWheelDriveMotorTorque(ii));
    }
    if(source_obj.GetWheelHasSteeringMotor(ii)) {
      hinge->setTargetVelocity(steering_motor_axis,source_obj.GetWheelSteeringMotorTargetVelocity(ii));
      hinge->setMaxMotorForce(steering_motor_axis,source_obj.GetWheelSteeringMotorTorque(ii));
    }
  }
  return dest_obj;
}

const btTransform& spBulletWorld::spPose2btTransform(const spPose& pose) {
  btTransform tr;
  tr.setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2]));
  spRotation q(pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  return tr;
}

// update all parameters of a box
int spBulletWorld::UpdateBulletBoxObject(spBox &source_obj, btRigidBody *dest_obj) {

  // if bullet pointer has not been defined yet then assign memory and initialize
  if (dest_obj == nullptr) {
    btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
    collisionShapes_.push_back(shape);
    btTransform tr;
    tr.setIdentity();
    dest_obj = CreateRigidBody(0,tr,shape);
    dynamics_world_->addRigidBody(dest_obj);
    dest_obj->setUserIndex(dynamics_world_->getNumCollisionObjects());
  }
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
//  tr.setOrigin(btVector3(source_obj.GetPose().translation()[0], source_obj.GetPose().translation()[1], source_obj.GetPose().translation()[2]));
//  Eigen::Quaterniond q(source_obj.GetPose().rotation());
//  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
//  dest_obj->setWorldTransform(tr);
  dest_obj->setWorldTransform(spPose2btTransform(source_obj.GetPose()));
  return dest_obj->getUserIndex();
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
          btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
          btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
          UpdateBulletBoxObject((spBox&)spobj,bulletbody);
          break;
      }
    }
  }
}

void spBulletWorld::UpdateSpiritObjectsFromPhy(Objects &spobjects) {
  for(int ii=0; ii<spobjects.GetNumOfObjects(); ii++) {
    //only update objects which are dynamic
    if(spobjects.GetObject(ii).IsDynamic()) {
      // get gui index of object
      int phy_index = spobjects.GetObject(ii).GetPhyIndex();
      // update the phy object
      btCollisionObject* obj = dynamics_world_->getCollisionObjectArray()[phy_index];
      btTransform bttrans;
      bttrans = obj->getWorldTransform();
      btVector3 btorigin = bttrans.getOrigin();
      spPose sppose(spPose::Identity());
      sppose.translate(spTranslation(btorigin[0],btorigin[1],btorigin[2]));
      btQuaternion btrot = bttrans.getRotation();
      spRotation spangle(btrot.w(),btrot.x(),btrot.y(),btrot.z());
      sppose.rotate(spangle);
      spobjects.GetObject(ii).SetPose(sppose);
    }
  }
}

void spBulletWorld::StepPhySimulation(double step_time) {
#warning "what is 10 here ..."
  dynamics_world_->stepSimulation(step_time,10);
}







