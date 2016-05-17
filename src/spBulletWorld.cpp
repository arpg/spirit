#include <spirit/Physics/spBulletWorld.h>

spBulletWorld::spBulletWorld() {
  world_params_.worldMax.setValue(-1000,-1000,-1000);
  world_params_.worldMin.setValue(1000,1000,1000);
  world_params_.has_defground = true;
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
  if(world_params_.has_defground) {
    delete(def_groundShape_);
  }

}

bool spBulletWorld::InitEmptyDynamicsWorld() {

  // Add a default ground if required
  if(world_params_.has_defground) {
    def_groundShape_ = new btBoxShape(btVector3(50,3,50));
    collisionShapes_.push_back(def_groundShape_);
  }

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
    default:
      std::cerr << "Error: olver has not been selected " << std::endl;
      return 0;
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

  dynamics_world_->getSolverInfo().m_numIterations = 100;
  return 1;
}

int spBulletWorld::AddBox(spBox& box) {

  btCollisionShape* shape = new btBoxShape(btVector3(box.dims[0],box.dims[1],box.dims[2]));
  collisionShapes_.push_back(shape);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (box.mass != 0.f);

  btVector3 localInertia(0,0,0);
  if (isDynamic) {
      // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
      shape->calculateLocalInertia(box.mass,localInertia);
  }

  btTransform tr;
  tr.setOrigin(btVector3(box.pose.translation()[0],box.pose.translation()[1],box.pose.translation()[2]));
  Eigen::Quaterniond q(box.pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
#ifdef USE_MOTIONSTATE
  btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  btRigidBody::btRigidBodyConstructionInfo cInfo(box.mass,myMotionState,shape,localInertia);
  btRigidBody* body = new btRigidBody(cInfo);
  //body->setContactProcessingThreshold(defaultContactProcessingThreshold_);
#else
	btRigidBody* body = new btRigidBody(box.mass,0,shape,localInertia);
	body->setWorldTransform(tr);
#endif

	dynamics_world_->addRigidBody(body);

	return body->getUserIndex();
}

int spBulletWorld::AddSphere(spSphere& sphere) {

  btCollisionShape* shape = new btSphereShape(sphere.radius);
  collisionShapes_.push_back(shape);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (sphere.mass != 0.f);

  btVector3 localInertia(0,0,0);
  if (isDynamic) {
      shape->calculateLocalInertia(sphere.mass,localInertia);
  }

  btTransform tr;
  tr.setOrigin(btVector3(sphere.pose.translation()[0],sphere.pose.translation()[1],sphere.pose.translation()[2]));
  Eigen::Quaterniond q(sphere.pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
#ifdef USE_MOTIONSTATE
  btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  btRigidBody::btRigidBodyConstructionInfo cInfo(sphere.mass,myMotionState,shape,localInertia);
  btRigidBody* body = new btRigidBody(cInfo);
  //body->setContactProcessingThreshold(defaultContactProcessingThreshold_);
#else
	btRigidBody* body = new btRigidBody(sphere.mass,0,shape,localInertia);
	body->setWorldTransform(tr);
#endif

	dynamics_world_->addRigidBody(body);

	return body->getUserIndex();
}

int spBulletWorld::AddCar(spCarParamseters& car_params) {
 std::cerr << "AddCar HAS NOT BEEN IMPLEMENTED!" << std::endl;
 return -1;
}





