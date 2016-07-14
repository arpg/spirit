#include <spirit/Physics/spBulletWorld.h>

spBulletWorld::spBulletWorld() {
  world_params_.worldMax.setValue(1000*WSCALE,1000*WSCALE,1000*WSCALE);
  world_params_.worldMin.setValue(-1000*WSCALE,-1000*WSCALE,-1000*WSCALE);
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
    case spObjectType::VEHICLE :
    {
      btRigidBody* thevehicle = CreateBulletVehicleObject((spVehicle&)sp_obj);
      UpdateBulletVehicleObject((spVehicle&)sp_obj,thevehicle);
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
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(0.1,0.2,0.05));
  collisionShapes_.push_back(chassis_shape);
  //cg_tr effectively shifts the center of mass with respect to the chassis
  // this transform is to put the cog in the right spot
//  spPose chassis_transform(spPose::Identity());
//  chassis_transform = source_obj.GetPose() * cog_transform.inverse();
  std::cout << "now doing chassis" << std::endl;
  std::cout << "cog_transform" << source_obj.GetLocalCOG().matrix() << std::endl;
  compound->addChildShape(spPose2btTransform(source_obj.GetLocalCOG().inverse(),WSCALE),chassis_shape);
//  compound->addChildShape(spPose2btTransform(spPose::Identity(),WSCALE),chassis_shape);  // test
  // create a rigidbody from compound shape and add it to world
  // apply cog transform
//  spPose global_cog(/*source_obj.GetPose() **/ cog_transform);
  btRigidBody* bodyA = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(source_obj.GetLocalCOG(),WSCALE),compound);
//  btRigidBody* bodyA = CreateRigidBody(source_obj.GetChassisMass(),spPose2btTransform(spPose::Identity(),WSCALE),compound);  // test
  dynamics_world_->addRigidBody(bodyA);
  // set the correct index for spVehicle object so we can access this object later
  bodyA->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
  source_obj.SetPhyIndex(bodyA->getUserIndex());
  // now create and add wheels
  for(int ii=0 ; ii<source_obj.GetNumberOfWheels() ; ii++) {
    spWheel* spwheel = source_obj.GetWheel(ii);
    bodyA->setActivationState(DISABLE_DEACTIVATION);
    // calculate wheel origin in world
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(source_obj.GetWheel(ii)->GetChassisAnchor()[0],source_obj.GetWheel(ii)->GetChassisAnchor()[1],source_obj.GetWheel(ii)->GetChassisAnchor()[2]-source_obj.GetWheel(ii)->GetSuspPreloadingSpacer())*WSCALE);
//    tr = spPose2btTransform(source_obj.GetWheel(ii)->GetPose());
    btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(0.04,0.05,0.05));
    btRigidBody* bodyB = CreateRigidBody(spwheel->GetMass(),tr,wheel_shape);
    bodyB->setDamping(0,0);
    dynamics_world_->addRigidBody(bodyB);
    bodyB->setUserIndex(dynamics_world_->getNumCollisionObjects()-1);
    spwheel->SetPhyIndex(bodyB->getUserIndex());
    bodyB->setFriction(spwheel->GetFriction());
    bodyB->setActivationState(DISABLE_DEACTIVATION);
    btVector3 parent_axis(0,0,1);
    btVector3 child_axis(1,0,0);
    btVector3 anchor = tr.getOrigin();
    btHinge2Constraint* hinge = new btHinge2Constraint(*bodyA,*bodyB,anchor, parent_axis, child_axis);
    // set suspension damping to axis 2 of constraint only (z direction)
//    hinge->setDamping(2,spwheel->GetSuspDamping());
//    hinge->setStiffness(2,spwheel->GetSuspStiffness());
    // fix x,y linear movement directions and only move in z direction
#warning "this whole function is missing world scaling, it might not be necesarry though since we update object right after we create it. then probably, not initializing is better than initializing in wrong scale"
    hinge->setLinearLowerLimit(btVector3(0,0,spwheel->GetSuspPreloadingSpacer()+spwheel->GetSuspLowerLimit()));
    hinge->setLinearUpperLimit(btVector3(0,0,spwheel->GetSuspUpperLimit()+spwheel->GetSuspPreloadingSpacer()));
    // set rotational directions
    // unlimitted in tire axis, fixed in one direction and limitted in steering direction(set upper/lower to 0/0 if its not supposed to be steering)
    hinge->setAngularLowerLimit(btVector3(1,0,spwheel->GetSteeringServoLowerLimit()));
    hinge->setAngularUpperLimit(btVector3(-1,0,spwheel->GetSteeringServoUpperLimit()));
    // add motors if required
    int drive_motor_axis = source_obj.GetWheel(ii)->GetDriveMotorAxis();
    int steering_servo_axis = source_obj.GetWheel(ii)->GetSteeringServoAxis();
    if(spwheel->GetHasDriveMotor()) {
      hinge->enableMotor(drive_motor_axis,true);
      hinge->setTargetVelocity(drive_motor_axis,spwheel->GetDriveMotorTargetVelocity());
      hinge->setMaxMotorForce(drive_motor_axis,spwheel->GetDriveMotorTorque());
    }
    if(spwheel->GetHasSteeringServo()) {
      // create a servo motor for this joint.
      hinge->enableMotor(steering_servo_axis,true);
      hinge->setTargetVelocity(steering_servo_axis,spwheel->GetSteeringServoMaxVelocity());
      hinge->setMaxMotorForce(steering_servo_axis,spwheel->GetSteeringServoTorque());
      hinge->setServo(steering_servo_axis,true);
      hinge->setServoTarget(steering_servo_axis,spwheel->GetSteeringServoTargetAngle());
    }
    // add the hinge constraint to the world and disable collision between bodyA/bodyB
    dynamics_world_->addConstraint(hinge,true);
  }
  return bodyA;
}


void spBulletWorld::UpdateBulletVehicleObject(spVehicle& source_obj, btRigidBody* dest_obj) {
  // get compoundshape from rigidbody
  btCompoundShape* compound = (btCompoundShape*) dest_obj->getCollisionShape();
  // since we only added chassis(box) to compound shape its gonna be in index_0 shape
  int box_childnumber = 0;
  btCollisionShape* chassis_shape = compound->getChildShape(box_childnumber);
//  std::cout << "comp tr" << btTransform2spPose(compound->getChildTransform(0),WSCALE_INV).matrix() << std::endl;
  // Here we should update all properties of the vehicle
  // reset box size
  spBoxSize dims(source_obj.GetChassisSize());
//  chassis_shape->setLocalScaling(btVector3(dims[0]/2,dims[1]/2,dims[2]/2)*WSCALE);
//  chassis_shape->setLocalScaling(btVector3(0.5,0.5,0.5)*WSCALE);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
//  btVector3 localInertia(0,0,0);
  // bullet calculates inertia tensor for a cuboid shape (it only has diagonal values).
//  chassis_shape->calculateLocalInertia(source_obj.GetChassisMass(),localInertia);
  // reset COG from spObject
//  dest_obj->setMassProps(source_obj.GetChassisMass(),localInertia);


//  spPose chassis_transform(spPose::Identity());
//  spPose cog_transform(spPose::Identity());
//  cog_transform.translate(source_obj.GetLocalCOG());
//  chassis_transform = source_obj.GetPose() * cog_transform.inverse();
//  compound->updateChildTransform(box_childnumber,spPose2btTransform(chassis_transform,WSCALE));
  dest_obj->setWorldTransform(spPose2btTransform(source_obj.GetPose()*source_obj.GetLocalCOG(),WSCALE));

  // Update wheel
  for(int ii=0; ii<source_obj.GetNumberOfWheels(); ii++){
    spWheel* spwheel = source_obj.GetWheel(ii);
    btRigidBody* wheel_body = &dest_obj->getConstraintRef(ii)->getRigidBodyB();
    // resize wheel
    btVector3 wheel_dim(spwheel->GetWidth(),spwheel->GetRadius(),spwheel->GetRadius());
//    wheel_body->getCollisionShape()->setLocalScaling(wheel_dim*WSCALE);
    // set wheel pose
#warning "why do we need to adjust wheels, it should be determined by phy engine"
    wheel_body->setWorldTransform(spPose2btTransform(spwheel->GetPose(),WSCALE));
    // calculate and set inertia/mass
    btVector3 wheel_local_inertia(0,0,0);
    double wheel_mass = spwheel->GetMass();
    wheel_body->getCollisionShape()->calculateLocalInertia(wheel_mass,wheel_local_inertia);
    wheel_body->setMassProps(wheel_mass,wheel_local_inertia);
    // reset wheel location
#warning "removed updating wheel origin, I guess this is something to be decided only by phy engine"
//    btTransform wheel_tr;
//    wheel_tr.setIdentity();
//    wheel_tr.setOrigin(btVector3(source_obj.GetWheel(ii)->GetChassisAnchor()[0],source_obj.GetWheel(ii)->GetChassisAnchor()[1],source_obj.GetWheel(ii)->GetChassisAnchor()[2]));
//    wheel_body->setWorldTransform(wheel_tr);
    // reset wheel friction and damping
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
      hinge->setMaxMotorForce(drive_motor_axis,spwheel->GetDriveMotorTorque());
    }
    if(spwheel->GetHasSteeringServo()) {
      // update
      hinge->setTargetVelocity(steering_servo_axis,spwheel->GetSteeringServoMaxVelocity());
      hinge->setMaxMotorForce(steering_servo_axis,spwheel->GetSteeringServoTorque());
      hinge->setServoTarget(steering_servo_axis,spwheel->GetSteeringServoTargetAngle());
    }
  }
}
#warning "returning bttransform might not be the fastest way of type conversion"
btTransform spBulletWorld::spPose2btTransform(const spPose& pose, double btworld_scale) {
  btTransform tr;
  tr.setOrigin(btVector3(pose.translation()[0],pose.translation()[1],pose.translation()[2])*btworld_scale);
  spRotation q(pose.rotation());
  tr.setRotation(btQuaternion(q.x(),q.y(),q.z(),q.w()));
  return tr;
}
#warning "if I return a reference instead of sppose UpdateSpiritObjectsFromPhy() will mess up things"
spPose spBulletWorld::btTransform2spPose(const btTransform& tr, double btworld_scale_inv) {
  spPose pose(spPose::Identity());
  btVector3 origin = tr.getOrigin();
  pose.translate(spTranslation(origin[0],origin[1],origin[2])*btworld_scale_inv);
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
        case spObjectType::VEHICLE:
        {
          btCollisionObject* col_obj = dynamics_world_->getCollisionObjectArray()[phy_index];
          btRigidBody* bulletbody = btRigidBody::upcast(col_obj);
          UpdateBulletVehicleObject((spVehicle&)spobj.GetObject(ii),bulletbody);
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
          spPose ps(btTransform2spPose(obj->getWorldTransform(),WSCALE_INV));
          box.SetPose(ps);
          break;
        }
        case spObjectType::VEHICLE:
        {
          spVehicle& vehicle = (spVehicle&) spobjects.GetObject(ii);
          // update chassis
          btCollisionObject* chassis_obj = dynamics_world_->getCollisionObjectArray()[vehicle.GetPhyIndex()];
          vehicle.SetPose(btTransform2spPose(chassis_obj->getWorldTransform(),WSCALE_INV)*vehicle.GetLocalCOG().inverse());
          // update wheels
          for(int ii=0; ii<vehicle.GetNumberOfWheels(); ii++) {
            btCollisionObject* wheel_obj = dynamics_world_->getCollisionObjectArray()[vehicle.GetWheel(ii)->GetPhyIndex()];
            vehicle.GetWheel(ii)->SetPose(btTransform2spPose(wheel_obj->getWorldTransform(),WSCALE_INV));
          }
          break;
        }
      }
    }
  }
}

void spBulletWorld::StepPhySimulation(double step_time) {
#warning "Is should read more about this step simulation function. increasing the last parameter avoid penetrations and also increases the processing load alot"
#warning "stepSimulation parameters are really important to understand since it changes simulation result drastically"
  // http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=&t=367
  // choose the number of iterations for the constraint solver in the range 4 to 10.
  dynamics_world_->stepSimulation(step_time,100,0.001);
}







