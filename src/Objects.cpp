#include <spirit/Objects.h>

Objects::Objects(){
  world_params_.worldMax.setValue(1000*WSCALE,1000*WSCALE,1000*WSCALE);
  world_params_.worldMin.setValue(-1000*WSCALE,-1000*WSCALE,-1000*WSCALE);
  world_params_.solver = spPhysolver::SEQUENTIAL_IMPULSE;
  this->InitEmptyDynamicsWorld();
}
Objects::~Objects(){
	//delete collision shapes
	for (int j=0;j<collisionShapes_.size();j++)
	{
		btCollisionShape* shape = collisionShapes_[j];
		delete shape;
	}

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
}


int Objects::CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color) {
  std::shared_ptr<spBox> a_box = std::make_shared<spBox>(pose,size,mass,color,dynamics_world_,collisionShapes_);
  objects_.push_back(a_box);
  return (objects_.size()-1);
}

int Objects::CreateWaypoint(const spPose& pose, const spColor& color) {
  std::shared_ptr<spWaypoint> a_waypoint = std::make_shared<spWaypoint>(pose,color);
  objects_.push_back(a_waypoint);
  return (objects_.size()-1);
}

int Objects::CreateVehicle(const spVehicleConstructionInfo& vehicle_info) {
  switch (vehicle_info.vehicle_type) {
    case spObjectType::VEHICLE_AWSD:
    {
      std::shared_ptr<spAWSDCar> a_vehicle = std::make_shared<spAWSDCar>(vehicle_info,dynamics_world_,collisionShapes_);
      objects_.push_back(a_vehicle);
      break;
    }
  }
  return (objects_.size()-1);
}

int Objects::CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color) {
  std::shared_ptr<spLineStrip> a_curve = std::make_shared<spLineStrip>(pose,linestrip_pts,color);
  objects_.push_back(a_curve);
  return (objects_.size()-1);
}


void Objects::RemoveObj(int obj_index) {
  if(obj_index<objects_.size()) {
    objects_.erase(objects_.begin()+obj_index);
  } else {
    SPERROREXIT("Requested Object doesn't exist.");
  }
}

spCommonObject& Objects::GetObject(int obj_index) {
  if(obj_index<objects_.size()) {
    return *objects_[obj_index].get();
  } else {
    SPERROREXIT("Requested Object doesn't exist.");
  }
}

int Objects::GetNumOfObjects() {
  return objects_.size();
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

