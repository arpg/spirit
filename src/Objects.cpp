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


int Objects::CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color) {
  std::shared_ptr<spBox> a_box = std::make_shared<spBox>(pose,size,mass,color,dynamics_world_);
  objects_.push_back(a_box);
  return (objects_.size()-1);
}
///////////////////////////////////////////////////////////////

btRigidBody* Objects::CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape) {
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

btTransform& Objects::spPose2btTransform(const spPose& pose, double btworld_scale) {
  std::shared_ptr<btTransform> tr = std::make_shared<btTransform>();
  tr->setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2])*btworld_scale);
  spRotation q(pose.rotation());
  tr->setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  return *tr.get();
}

spPose& Objects::btTransform2spPose(const btTransform& tr, double btworld_scale_inv) {
  std::shared_ptr<spPose> pose = std::make_shared<spPose>(spPose::Identity());
  btVector3 origin = tr.getOrigin();
  pose->translate(spTranslation(origin[0],origin[1],origin[2])*btworld_scale_inv);
  btQuaternion btrot = tr.getRotation();
  spRotation spangle(btrot.w(),btrot.x(),btrot.y(),btrot.z());
  pose->rotate(spangle);
  return *pose.get();
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
      btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
      std::shared_ptr<spAWSDCar> a_vehicle = std::make_shared<spAWSDCar>(vehicle_info,dynamics_world_);
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

