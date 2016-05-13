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


void spBulletWorld::AddRigidBody(spRigidBodyType& rb_type) {
}

void spBulletWorld::AddRigidBody(spRigidBodyType rb_type, double mass, const spBoxSize& box_size ,const spPose& pose) {
  btCollisionShape* shape;
  switch(rb_type) {
    case BOX:
      shape = new btBoxShape(btVector3(box_size[0],box_size[1],box_size[2]));
      break;
  }

  collisionShapes_.push_back(shape);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0,0,0);
  if (isDynamic)
    shape->calculateLocalInertia(mass,localInertia);

  btTransform tr;
  tr.setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2]));
//  tr.setRotation(btQuaternion(pose.rot));
  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
#ifdef USE_MOTIONSTATE
  btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
  btRigidBody* body = new btRigidBody(cInfo);
  //body->setContactProcessingThreshold(defaultContactProcessingThreshold_);
#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif

	dynamics_world_->addRigidBody(body);


}













