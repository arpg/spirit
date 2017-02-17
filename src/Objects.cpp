#include <spirit/Objects.h>

Objects::Objects(){
  world_params_.worldMax.setValue(1000*WSCALE,1000*WSCALE,1000*WSCALE);
  world_params_.worldMin.setValue(-1000*WSCALE,-1000*WSCALE,-1000*WSCALE);
  world_params_.solver = spPhysolver::SEQUENTIAL_IMPULSE;
  this->InitEmptyDynamicsWorld();
}
Objects::~Objects(){
	//delete collision shapes
//	for (int j=0;j<collisionShapes_.size();j++)
//	{
//		btCollisionShape* shape = collisionShapes_[j];
//		delete shape;
//	}
  spObjectHandle ii = objects_.begin();
  while(ii != objects_.end()){
    RemoveObj(ii);
    ii = objects_.begin();
  }
//  for (spObjectHandle ii=objects_.begin();ii!=objects_.end();++ii) {
//    RemoveObj(--ii);
//  }
  delete(dynamics_world_);
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZIG:
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

void Objects::InitEmptyDynamicsWorld() {

  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
  broadphase_ = new btAxisSweep3(world_params_.worldMin,world_params_.worldMax);
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZIG:
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
    case spPhysolver::MLCP_DANTZIG:
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

  dynamics_world_->setGravity(btVector3(0,0,-9.80665)*WSCALE);
  dynamics_world_->getSolverInfo().m_numIterations = BULLET_SOLVER_NUM_ITERATIONS;
//  collisionShapes_.clear();
  dynamics_world_->clearForces();
}


spObjectHandle Objects::CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color) {
  std::shared_ptr<spBox> a_box = std::make_shared<spBox>(pose,size,mass,color,dynamics_world_);
  objects_.push_back(a_box);
  return (--objects_.end());
}

spObjectHandle Objects::CreateWaypoint(const spPose& pose, const spColor& color) {
  std::shared_ptr<spWaypoint> a_waypoint = std::make_shared<spWaypoint>(pose,color);
  objects_.push_back(a_waypoint);
  return (--objects_.end());
}

spObjectHandle Objects::CreateVehicle(const spVehicleConstructionInfo& vehicle_info) {
  switch (vehicle_info.vehicle_type) {
    case spObjectType::VEHICLE_AWSD:
    {
      btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
      std::shared_ptr<spAWSDCar> a_vehicle = std::make_shared<spAWSDCar>(vehicle_info,dynamics_world_);
      objects_.push_back(a_vehicle);
      break;
    }
    default:
      std::cout << "this is not supported" << std::endl;
  }
  return (--objects_.end());
}

spObjectHandle Objects::CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color) {
  std::shared_ptr<spLineStrip> a_curve = std::make_shared<spLineStrip>(pose,linestrip_pts,color);
  objects_.push_back(a_curve);
  return (--objects_.end());
}

void Objects::RemoveObj(spObjectHandle& obj_handle) {
  if(obj_handle != NULL_HANDLE) {
    if(GetObject(obj_handle).GetObjecType() == spObjectType::VEHICLE_AWSD) {
      spAWSDCar& car = (spAWSDCar&) GetObject(obj_handle);
      dynamics_world_->removeRigidBody(car.GetWheel(3)->GetRigidbody());
      dynamics_world_->removeConstraint(car.GetWheel(3)->GetRigidbody()->getConstraintRef(0));
      dynamics_world_->removeRigidBody(car.GetWheel(2)->GetRigidbody());
      dynamics_world_->removeConstraint(car.GetWheel(2)->GetRigidbody()->getConstraintRef(0));
      dynamics_world_->removeRigidBody(car.GetWheel(1)->GetRigidbody());
      dynamics_world_->removeConstraint(car.GetWheel(1)->GetRigidbody()->getConstraintRef(0));
      dynamics_world_->removeRigidBody(car.GetWheel(0)->GetRigidbody());
      dynamics_world_->removeConstraint(car.GetWheel(0)->GetRigidbody()->getConstraintRef(0));
      dynamics_world_->removeRigidBody(car.GetRigidbody());
      dynamics_world_->clearForces();
    } else if(GetObject(obj_handle).GetObjecType() == spObjectType::BOX) {
      spBox& box = (spBox&) GetObject(obj_handle);
      dynamics_world_->removeRigidBody(box.GetRigidbody());
      dynamics_world_->clearForces();
    } else {
      SPERROREXIT("Removing other objects not implemented.");
    }
    objects_.erase(obj_handle);
    obj_handle = NULL_HANDLE;
  } else {
    SPERROREXIT("Requested Handle is NULL.");
  }
}

spCommonObject& Objects::GetObject(spObjectHandle& obj_handle) {
  if(obj_handle != NULL_HANDLE) {
    return *(obj_handle->get());
  } else {
    SPERROREXIT("Requested Object doesn't exist.");
  }
}

int Objects::GetNumOfObjects() {
  return objects_.size();
}

spObjectHandle Objects::GetListBegin() {
  return objects_.begin();
}

spObjectHandle Objects::GetListEnd() {
  return objects_.end();
}


void Objects::StepPhySimulation(double step_time) {
  // simulation_step/penetration -> http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=&t=367
  // http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_the_World
  // choose the number of iterations for the constraint solver in the range 4 to 10.
  const btScalar fixed_time_step = 0.001;
  if (step_time < fixed_time_step) {
    SPERROREXIT("step_time should be greater than fixed_time_step in Line:");
  }
  // this is to guarantee that all steps are simulation steps rather than interpolation
  // timeStep < maxSubSteps * fixedTimeStep
  int max_sub_steps = (step_time/fixed_time_step)+1;
  dynamics_world_->stepSimulation(step_time,max_sub_steps,fixed_time_step);
}

