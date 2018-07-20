#include <spirit/Objects.h>

Objects::Objects(){
  world_params_.worldMax.setValue(1000*WSCALE,1000*WSCALE,1000*WSCALE);
  world_params_.worldMin.setValue(-1000*WSCALE,-1000*WSCALE,-1000*WSCALE);
  world_params_.solver = spPhysolver::SEQUENTIAL_IMPULSE;
  this->InitEmptyDynamicsWorld();
}

Objects::Objects(const Objects& obj) {
  SPERROREXIT("Objects cpyConstructor shouldn't be called");
}

Objects::~Objects(){
  spObjectHandle ii = objects_.begin();
  while(ii != objects_.end()){
    RemoveObj(ii);
    ii = objects_.begin();
  }
//  for (spObjectHandle ii=objects_.begin();ii!=objects_.end();++ii) {
//    RemoveObj(--ii);
//  }
  dynamics_world_->clearForces();
  dynamics_world_.reset();
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZIG:
      solver_dantzig_.reset();
      break;
    case spPhysolver::MLCP_PROJECTEDGAUSSSEIDEL:
      solver_gseidel_.reset();
      break;
    case spPhysolver::SEQUENTIAL_IMPULSE:
      // do nothing
      break;
  }
  solver_.reset();
  broadphase_.reset();
  dispatcher_.reset();
  collisionConfiguration_.reset();
}

void Objects::InitEmptyDynamicsWorld() {

  collisionConfiguration_ = std::make_shared<btDefaultCollisionConfiguration>();
  dispatcher_ = std::make_shared<btCollisionDispatcher>(collisionConfiguration_.get());
  broadphase_ = std::make_shared<btAxisSweep3>(world_params_.worldMin,world_params_.worldMax);
  switch(world_params_.solver) {
    case spPhysolver::MLCP_DANTZIG:
      solver_dantzig_ = std::make_shared<btDantzigSolver>();
      solver_ = std::make_shared<btMLCPSolver>(solver_dantzig_.get());
      break;
    case spPhysolver::MLCP_PROJECTEDGAUSSSEIDEL:
      solver_gseidel_ = std::make_shared<btSolveProjectedGaussSeidel>();
      solver_ = std::make_shared<btMLCPSolver>(solver_gseidel_.get());
      break;
    case spPhysolver::SEQUENTIAL_IMPULSE:
      solver_ = std::make_shared<btSequentialImpulseConstraintSolver>();
      break;
  }

  dynamics_world_ = std::make_shared<btDiscreteDynamicsWorld>(dispatcher_.get(),broadphase_.get(),solver_.get(),collisionConfiguration_.get());

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
    case spObjectType::VEHICLE_BIKE:
    {
       btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
       std::shared_ptr<spBike> a_vehicle = std::make_shared<spBike>(vehicle_info,dynamics_world_);
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
spObjectHandle Objects::CreateLineStrip(const spPose& pose, const spCurve& curve, int num_pts, const spColor& color) {
  std::shared_ptr<spLineStrip> a_curve = std::make_shared<spLineStrip>(pose,curve,num_pts,color);
  objects_.push_back(a_curve);
  return (--objects_.end());
}

void Objects::RemoveObj(spObjectHandle& obj_handle) {
  if(obj_handle != NULL_HANDLE) {
    if(GetObject(obj_handle).GetObjecType() == spObjectType::VEHICLE_AWSD) {
      spAWSDCar& car = (spAWSDCar&) GetObject(obj_handle);
      for(int ii=car.GetNumberOfWheels()-1; ii>=0; ii--) {
        dynamics_world_->removeRigidBody(car.GetWheel(ii)->GetRigidbody().get());
        dynamics_world_->removeConstraint(car.GetWheel(ii)->GetRigidbody().get()->getConstraintRef(0));
      }
      dynamics_world_->removeRigidBody(car.GetRigidbody().get());
      dynamics_world_->clearForces();
    } else if(GetObject(obj_handle).GetObjecType() == spObjectType::BOX) {
      spBox& box = (spBox&) GetObject(obj_handle);
      dynamics_world_->removeRigidBody(box.GetRigidbody().get());
      dynamics_world_->clearForces();
    } else {
      if(GetObject(obj_handle).GetObjecType() == spObjectType::WHEEL)
        SPERROR("obj is wheel");
      if(GetObject(obj_handle).GetObjecType() == spObjectType::WAYPOINT)
        SPERROR("obj is waypoint");
      if(GetObject(obj_handle).GetObjecType() == spObjectType::LINESTRIP)
        SPERROR("obj is linestrip");
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

