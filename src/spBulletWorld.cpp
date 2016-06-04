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

void spBulletWorld::AddNewPhyObject(spCommonObject &sp_obj) {
  switch (sp_obj.GetObjecType()) {
    case spObjectType::BOX:
      // pass in nullptr to create a new bullet object
      btRigidBody* body = UpdateBulletBoxObject((spBox&)sp_obj,nullptr);
      body->setUserIndex(dynamics_world_->getNumCollisionObjects());
      dynamics_world_->addRigidBody(body);
      sp_obj.SetPhyIndex(body->getUserIndex());
      break;
    case spObjectType::CAR:
      // pass in nullptr to create a new bullet object
      btRigidBody* compound_body = UpdateBulletBoxObject((spCar&)sp_obj,nullptr);
      compound_body->setUserIndex(dynamics_world_->getNumCollisionObjects());
      dynamics_world_->addRigidBody(compound_body);
      sp_obj.SetPhyIndex(compound_body->getUserIndex());
      break;
  }
}

// update all parameters of a box
btRigidBody* spBulletWorld::UpdateBulletBoxObject(spBox &source_obj, btRigidBody *dest_obj) {
  // if bullet pointer has not been defined yet then assign memory and initialize
  if (dest_obj == nullptr) {
    btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
    collisionShapes_.push_back(shape);
    btDefaultMotionState* motion_state = new btDefaultMotionState;
    btRigidBody::btRigidBodyConstructionInfo cInfo(0,motion_state,shape,btVector3(0,0,0));
    dest_obj = new btRigidBody(cInfo);
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
  btTransform tr;
  tr.setOrigin(btVector3(source_obj.GetPose().translation()[0], source_obj.GetPose().translation()[1], source_obj.GetPose().translation()[2]));
  Eigen::Quaterniond q(source_obj.GetPose().rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  dest_obj->setWorldTransform(tr);
  return dest_obj;
}

btRigidBody* spBulletWorld::UpdateBulletCarObject(spCar& source_obj, btRigidBody* dest_obj) {
  // if bullet pointer has not been defined yet then assign memory and initialize
  if (dest_obj == nullptr) {
    btCollisionShape* chassis_shape = new btBoxShape(btVector3(1.f,0.5f,2.f));
    collisionShapes_.push_back(chassis_shape);
    btCompoundShape compound = new btCompoundShape();
    collisionShapes_.push_back(compound);
    //chassis_tr effectively shifts the center of mass with respect to the chassis
    btTransform chassis_tr;
    chassis_tr.setIdentity();
    chassis_tr.setOrigin(btVector3(0,1,0));
    compound.addChildShape(chassis_tr,chassis_shape);

    btDefaultMotionState* motion_state = new btDefaultMotionState;
    btRigidBody::btRigidBodyConstructionInfo cInfo(0,motion_state,shape,btVector3(0,0,0));
    dest_obj = new btRigidBody(cInfo);
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
  btTransform tr;
  tr.setOrigin(btVector3(source_obj.GetPose().translation()[0], source_obj.GetPose().translation()[1], source_obj.GetPose().translation()[2]));
  Eigen::Quaterniond q(source_obj.GetPose().rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  dest_obj->setWorldTransform(tr);
  return dest_obj;
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
#warning  "upcasting might not pass pointer correctly, this should be tested"
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
  dynamics_world_->stepSimulation(step_time,10);
}







