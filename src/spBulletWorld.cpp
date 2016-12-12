#include <spirit/Physics/spBulletWorld.h>
// Bullet spring model Ref  -->  http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=9997


spBulletWorld::spBulletWorld() {
  world_params_.worldMax.setValue(1000*WSCALE,1000*WSCALE,1000*WSCALE);
  world_params_.worldMin.setValue(-1000*WSCALE,-1000*WSCALE,-1000*WSCALE);
  world_params_.solver = spPhysolver::SEQUENTIAL_IMPULSE;
  chassis_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
  wheel_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH;
  box_collides_with_ = BulletCollissionType::COL_BOX | BulletCollissionType::COL_MESH | BulletCollissionType::COL_CHASSIS | BulletCollissionType::COL_WHEEL;
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

bool spBulletWorld::InitEmptyDynamicsWorld() {

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
    case spObjectType::VEHICLE_AWD||spObjectType::VEHICLE_AWSD||spObjectType::VEHICLE_GENERAL || spObjectType::VEHICLE_RWD :
    {
      btRigidBody* thevehicle = CreateBulletVehicleObject((spVehicle&)sp_obj);
      UpdateBulletVehicleObject((spVehicle&)sp_obj,thevehicle);
      break;
    }
    case spObjectType::WHEEL:
    {
      std::cerr << "WHEEL should not be implemented by itself" << std::endl;
      break;
    }
    case spObjectType::WAYPOINT:
    {
      std::cerr << "WAYPOINT should not be implemented by itself" << std::endl;
      break;
    }
  }
}

btRigidBody* spBulletWorld::CreateBulletVehicleObject(spVehicle& source_obj) {
  // Here we are gonna create a bullet vehicle from scratch
  // Create a bullet compound shape
  btCompoundShape* compound = new btCompoundShape();
  collisionShapes_.push_back(compound);
  // add chassis as a box to compound shape
  spBoxSize chassis_size(source_obj.GetChassisSize());
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(chassis_size[0]/2,chassis_size[1]/2,chassis_size[2]/2)*WSCALE);
  collisionShapes_.push_back(chassis_shape);
  // this transform is to put the cog in the right spot
  compound->addChildShape(spPose2btTransform(source_obj.GetLocalCOG().inverse(),WSCALE),chassis_shape);
  // create a rigidbody from compound shape and add it to world
  btRigidBody* bodyA = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(source_obj.GetLocalCOG(),WSCALE),compound);
  dynamics_world_->addRigidBody(bodyA,BulletCollissionType::COL_CHASSIS,chassis_collides_with_);
  // set the correct index for spVehicle object so we can access this object later
  bodyA->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  source_obj.SetPhyIndex(bodyA->getUserIndex());
  // set body velocities to zero
  bodyA->setLinearVelocity(btVector3(0,0,0));
  bodyA->setAngularVelocity(btVector3(0,0,0));
  // set damping to zero since we are moving in air
  bodyA->setDamping(0,0);
  // now create and add wheels
  for(int ii=0 ; ii<source_obj.GetNumberOfWheels() ; ii++) {
    spWheel* spwheel = source_obj.GetWheel(ii);
    bodyA->setActivationState(DISABLE_DEACTIVATION);
    // calculate wheel origin in world
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(source_obj.GetWheel(ii)->GetChassisAnchor()[0],source_obj.GetWheel(ii)->GetChassisAnchor()[1],source_obj.GetWheel(ii)->GetChassisAnchor()[2]-source_obj.GetWheel(ii)->GetSuspPreloadingSpacer())*WSCALE);
//    btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(source_obj.GetWheel(ii)->GetWidth()/2,source_obj.GetWheel(ii)->GetRadius(),source_obj.GetWheel(ii)->GetRadius())*WSCALE);
    btCollisionShape* wheel_shape = new btCapsuleShapeX(source_obj.GetWheel(ii)->GetRadius()*WSCALE,(source_obj.GetWheel(ii)->GetWidth()/2)*WSCALE);
    btRigidBody* bodyB = CreateRigidBody(spwheel->GetMass(),tr,wheel_shape);
    bodyB->setDamping(0.0,0.0);
    dynamics_world_->addRigidBody(bodyB,BulletCollissionType::COL_WHEEL,wheel_collides_with_);
    bodyB->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
    spwheel->SetPhyIndex(bodyB->getUserIndex());
    bodyB->setRollingFriction(spwheel->GetFriction());
    bodyB->setActivationState(DISABLE_DEACTIVATION);
    btVector3 parent_axis(0,0,1);
    btVector3 child_axis(1,0,0);
    btVector3 anchor = tr.getOrigin();
    btHinge2Constraint* hinge = new btHinge2Constraint(*bodyA,*bodyB,anchor, parent_axis, child_axis);
    // set suspension damping to axis 2 of constraint only (z direction)
    hinge->setDamping(2,spwheel->GetSuspDamping());
    hinge->setStiffness(2,spwheel->GetSuspStiffness());
    // fix x,y linear movement directions and only move in z direction
    hinge->setLinearLowerLimit(btVector3(0,0,spwheel->GetSuspPreloadingSpacer()+spwheel->GetSuspLowerLimit())*WSCALE);
    hinge->setLinearUpperLimit(btVector3(0,0,spwheel->GetSuspPreloadingSpacer()+spwheel->GetSuspUpperLimit())*WSCALE);
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
    hinge->setAngularLowerLimit(btVector3(1,0,spwheel->GetSteeringServoLowerLimit()));
    hinge->setAngularUpperLimit(btVector3(-1,0,spwheel->GetSteeringServoUpperLimit()));
    // add motors if required
    int drive_motor_axis = source_obj.GetWheel(ii)->GetDriveMotorAxis();
    int steering_servo_axis = source_obj.GetWheel(ii)->GetSteeringServoAxis();
    if(spwheel->GetHasDriveMotor()) {
      hinge->enableMotor(drive_motor_axis,true);
      hinge->setTargetVelocity(drive_motor_axis,spwheel->GetDriveMotorTargetVelocity()/**WSCALE*/);
      hinge->setMaxMotorForce(drive_motor_axis,spwheel->GetDriveMotorTorque()*WSCALE*WSCALE);
    }
    if(spwheel->GetHasSteeringServo()) {
      // create a servo motor for this joint.
      hinge->enableMotor(steering_servo_axis,true);
      hinge->setTargetVelocity(steering_servo_axis,spwheel->GetSteeringServoMaxVelocity()/**WSCALE*/);
      hinge->setMaxMotorForce(steering_servo_axis,spwheel->GetSteeringServoTorque()*WSCALE*WSCALE);
      hinge->setServo(steering_servo_axis,true);
      hinge->setServoTarget(steering_servo_axis,spwheel->GetSteeringServoTargetAngle());
    }
    // add the hinge constraint to the world and disable collision between bodyA/bodyB
    dynamics_world_->addConstraint(hinge,true);
  }
  return bodyA;
}


void spBulletWorld::UpdateBulletVehicleObject(spVehicle& source_obj, btRigidBody* dest_obj) {
  // update phy object in the case spirit objects has been changed anywhere other than phy engine
  // get compoundshape from rigidbody
  btCompoundShape* compound = (btCompoundShape*) dest_obj->getCollisionShape();
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
  compound->calculateLocalInertia(source_obj.GetChassisMass(),localInertia);
  dest_obj->setMassProps(source_obj.GetChassisMass(),localInertia);
  // Move chassis to most recent pose
  dest_obj->setWorldTransform(spPose2btTransform(source_obj.GetPose()*source_obj.GetLocalCOG(),WSCALE));
  // set default gravity
  dest_obj->setGravity(dynamics_world_->getGravity());
  // set air damping coefficient to 0
  dest_obj->setDamping(0,0);
  // set chassis friction
  dest_obj->setFriction(source_obj.GetFriction());
  dest_obj->setRollingFriction(source_obj.GetRollingFriction());

  // Update wheel
  for(int ii=0; ii<source_obj.GetNumberOfWheels(); ii++){
    spWheel* spwheel = source_obj.GetWheel(ii);
    btRigidBody* wheel_body = &dest_obj->getConstraintRef(ii)->getRigidBodyB();
    // set wheen gravity
    wheel_body->setGravity(dynamics_world_->getGravity());
    // set wheel pose. since we have moved the chassis, wheels need to move too
    wheel_body->setWorldTransform(spPose2btTransform(spwheel->GetPose(),WSCALE));

    // calculate and set inertia/mass
    btVector3 wheel_local_inertia(0,0,0);
    double wheel_mass = spwheel->GetMass();
    wheel_body->getCollisionShape()->calculateLocalInertia(wheel_mass,wheel_local_inertia);
    wheel_body->setMassProps(wheel_mass,wheel_local_inertia);
    // reset wheel friction and damping
    wheel_body->setRollingFriction(spwheel->GetRollingFriction());
    wheel_body->setFriction(spwheel->GetFriction());
    // reset suspension damping to 2-axis of constraint only (z direction)
    btHinge2Constraint* hinge = (btHinge2Constraint*) dest_obj->getConstraintRef(ii);
    hinge->setDamping(2,spwheel->GetSuspDamping());
    hinge->setStiffness(2,spwheel->GetSuspStiffness());
    // fix x,y linear movement directions and only move in z direction
    hinge->setLinearLowerLimit(btVector3(0,0,spwheel->GetSuspPreloadingSpacer()+spwheel->GetSuspLowerLimit())*WSCALE);
    hinge->setLinearUpperLimit(btVector3(0,0,spwheel->GetSuspPreloadingSpacer()+spwheel->GetSuspUpperLimit())*WSCALE);
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
    hinge->setAngularLowerLimit(btVector3(1,0,spwheel->GetSteeringServoLowerLimit()));
    hinge->setAngularUpperLimit(btVector3(-1,0,spwheel->GetSteeringServoUpperLimit()));
    // add motors if required
    int drive_motor_axis = source_obj.GetWheel(ii)->GetDriveMotorAxis();
    int steering_servo_axis = source_obj.GetWheel(ii)->GetSteeringServoAxis();
    if(spwheel->GetHasDriveMotor()) {
      hinge->setTargetVelocity(drive_motor_axis,spwheel->GetDriveMotorTargetVelocity());
      hinge->setMaxMotorForce(drive_motor_axis,spwheel->GetDriveMotorTorque()*WSCALE*WSCALE);
    }
    if(spwheel->GetHasSteeringServo()) {
      hinge->setTargetVelocity(steering_servo_axis,spwheel->GetSteeringServoMaxVelocity());
      hinge->setMaxMotorForce(steering_servo_axis,spwheel->GetSteeringServoTorque()*WSCALE*WSCALE);
      hinge->setServoTarget(steering_servo_axis,spwheel->GetSteeringServoTargetAngle());
    }
  }
}

btTransform& spBulletWorld::spPose2btTransform(const spPose& pose, double btworld_scale) {
  std::shared_ptr<btTransform> tr = std::make_shared<btTransform>();
  tr->setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2])*btworld_scale);
  spRotation q(pose.rotation());
  tr->setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  return *tr.get();
}

spPose& spBulletWorld::btTransform2spPose(const btTransform& tr, double btworld_scale_inv) {
  std::shared_ptr<spPose> pose = std::make_shared<spPose>(spPose::Identity());
  btVector3 origin = tr.getOrigin();
  pose->translate(spTranslation(origin[0],origin[1],origin[2])*btworld_scale_inv);
  btQuaternion btrot = tr.getRotation();
  spRotation spangle(btrot.w(),btrot.x(),btrot.y(),btrot.z());
  pose->rotate(spangle);
  return *pose.get();
}


btRigidBody* spBulletWorld::CreateBulletBoxObject(spBox &source_obj) {
  btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
  collisionShapes_.push_back(shape);
  btTransform tr;
  tr.setIdentity();
  btRigidBody* dest_obj = CreateRigidBody(1,tr,shape);
  dynamics_world_->addRigidBody(dest_obj,BulletCollissionType::COL_BOX,box_collides_with_);
  dest_obj->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  dest_obj->setActivationState(DISABLE_DEACTIVATION);
  source_obj.SetPhyIndex(dest_obj->getUserIndex());
  return dest_obj;
}


// update all parameters of a box
void spBulletWorld::UpdateBulletBoxObject(spBox &source_obj, btRigidBody *dest_obj) {
  // reset box size
  spBoxSize dims(source_obj.GetDimensions());
  dest_obj->getCollisionShape()->setLocalScaling(btVector3(dims[0]/2,dims[1]/2,dims[2]/2)*WSCALE);
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  btVector3 localInertia(0,0,0);
  if (source_obj.IsDynamic()) {
      // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
      dest_obj->getCollisionShape()->calculateLocalInertia(source_obj.GetMass(),localInertia);
  }
  // reset object mass
  dest_obj->setMassProps(source_obj.GetMass(),localInertia);
  // transform phy object
  dest_obj->setWorldTransform(spPose2btTransform(source_obj.GetPose(),WSCALE));
  dest_obj->setGravity(dynamics_world_->getGravity());
  dest_obj->setFriction(source_obj.GetFriction());
  dest_obj->setRollingFriction(source_obj.GetRollingFriction());
}

// This goes through objects and if they require clamping to surface it moves the object in given direction until it hits a surface
// this will also align the object such that it sits on surface stable
// freez all dynamic objects other than the ones which are gonna be clamped then simulate forward and then unfreez them all
void spBulletWorld::ClampObjectsToSurfaces(Objects &spobj) {
  bool sim_required = false;
  btRigidBody* bulletbody;
  // go through all spirit objects
  for(int ii=0; ii<spobj.GetNumOfObjects(); ii++) {
    //only update objects which had physics property changes
    if(spobj.GetObject(ii).IsDynamic()) {
      int phy_index = spobj.GetObject(ii).GetPhyIndex();
      btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
      bulletbody = btRigidBody::upcast(col_obj);
      if(!spobj.GetObject(ii).NeedsClampToSurface()) {
        // fix the objects which don't need to be clamped
        bulletbody->forceActivationState(DISABLE_SIMULATION);
      } else {
        sim_required = true;
        bulletbody->setDamping(10,10);
        switch (spobj.GetObject(ii).GetObjecType()) {
          case spObjectType::VEHICLE_AWD||spObjectType::VEHICLE_AWSD||spObjectType::VEHICLE_GENERAL || spObjectType::VEHICLE_RWD :
          {
            std::cout << "now clamping the car" << std::endl;
            spVehicle& vehicle = (spVehicle&) spobj.GetObject(ii);
            for(int jj=0; jj<vehicle.GetNumberOfWheels(); jj++) {
              btHinge2Constraint* hinge = (btHinge2Constraint*) bulletbody->getConstraintRef(ii);
              hinge->setTargetVelocity(0,0);
              hinge->setTargetVelocity(1,0);
              hinge->setTargetVelocity(2,0);
              hinge->setTargetVelocity(3,0);
              hinge->setTargetVelocity(4,0);
              hinge->setTargetVelocity(5,0);
            }
            break;
          }
        }
      }
    }
  }
  // run simulation to clamp the object to surface
  if(sim_required) {
    for(int ii=0;ii<20;ii++) {
      this->StepPhySimulation(0.001);
    }
  }
  // update the bullet object with its spirit object
  for(int ii=0; ii<spobj.GetNumOfObjects(); ii++) {
    //only update objects which had physics property changes
    if(spobj.GetObject(ii).IsDynamic()) {
      int phy_index = spobj.GetObject(ii).GetPhyIndex();
      btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
      btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
      if(!spobj.GetObject(ii).NeedsClampToSurface()) {
        // enable back this object
        bulletbody->forceActivationState(DISABLE_DEACTIVATION);
      } else {
        bulletbody->setDamping(0,0);
        spobj.GetObject(ii).SetClamped();
      }
    }
  }
  if(sim_required) {
    UpdateSpiritObjectsFromPhy(spobj);
  }
}

void spBulletWorld::UpdatePhyObjectsFromSpirit(Objects &spobj) {
  // go through all spirit objects
  for(int ii=0; ii<spobj.GetNumOfObjects(); ii++) {
    //only update objects which had physics property changes
    if(spobj.GetObject(ii).HasChangedPhy()) {
      // get phy index of object
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
        case spObjectType::VEHICLE_AWD||spObjectType::VEHICLE_AWSD||spObjectType::VEHICLE_GENERAL || spObjectType::VEHICLE_RWD :
        {
          btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
          btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
          UpdateBulletVehicleObject((spVehicle&)spobj.GetObject(ii),bulletbody);
          spobj.GetObject(ii).SetPhyUpdated();
          break;
        }
        case spObjectType::WHEEL:
        {
          SPERROREXIT("WHEEL should not be implemented by itself");
          break;
        }
        case spObjectType::WAYPOINT:
        {
          SPERROREXIT("WAYPOINT should not be implemented in phy");
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
          spPose ps(btTransform2spPose(obj->getWorldTransform(),WSCALE_INV));
          box.SetPose(ps);
          break;
        }
        case spObjectType::VEHICLE_AWD||spObjectType::VEHICLE_AWSD||spObjectType::VEHICLE_GENERAL || spObjectType::VEHICLE_RWD :
        {
          spVehicle& vehicle = (spVehicle&) spobjects.GetObject(ii);
          // update chassis
          btCollisionObject* chassis_obj = dynamics_world_->getCollisionObjectArray()[vehicle.GetPhyIndex()];
          vehicle.SetPose(btTransform2spPose(chassis_obj->getWorldTransform(),WSCALE_INV)*vehicle.GetLocalCOG().inverse());
          // update chassis linear and angular velocities
          btRigidBody* chassis_rigbody = btRigidBody::upcast(chassis_obj);
          chassis_rigbody->setCollisionShape(chassis_obj->getCollisionShape());
          spVelocity spvel;
          spvel << chassis_rigbody->getLinearVelocity()[0],chassis_rigbody->getLinearVelocity()[1],chassis_rigbody->getLinearVelocity()[2],
              chassis_rigbody->getAngularVelocity()[0],chassis_rigbody->getAngularVelocity()[1],chassis_rigbody->getAngularVelocity()[2];
          vehicle.SetVelocity(spvel);

          // update wheels
          for(int ii=0; ii<vehicle.GetNumberOfWheels(); ii++) {
            btCollisionObject* wheel_obj = dynamics_world_->getCollisionObjectArray()[vehicle.GetWheel(ii)->GetPhyIndex()];
            vehicle.GetWheel(ii)->SetPose(btTransform2spPose(wheel_obj->getWorldTransform(),WSCALE_INV));
          }
          break;
        }
        case spObjectType::WHEEL:
        {
          SPERROREXIT("WHEEL should not be implemented by itself");
          break;
        }
        case spObjectType::WAYPOINT:
        {
          SPERROREXIT("WAYPOINT should not be implemented in phy");
          break;
        }
      }
    }
  }
}

void spBulletWorld::StepPhySimulation(double step_time) {

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







