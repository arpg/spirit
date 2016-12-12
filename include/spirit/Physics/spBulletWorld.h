#ifndef SP_BULLETWORLD_H__
#define SP_BULLETWORLD_H__

#warning "Comment the following line if using bullet in single precision mode"
#define BT_USE_DOUBLE_PRECISION

#include <iostream>
#include <spirit/Physics/spPhysicsWorld.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.h>


class spBulletWorld: public spPhysicsWorld {

// for a stable physics result use a scale of 2-10
#define WSCALE 10
#define WSCALE_INV 0.1

struct BulletWorldParams{
  btVector3 worldMin;
  btVector3 worldMax;
  spPhysolver solver;
};

#define BIT(x) (1<<(x))
enum BulletCollissionType{
  COL_NOTHING = 0,      // Collide with nothing
  COL_BOX = BIT(0),     // Collide with box
  COL_MESH = BIT(1),    // Collide with mesh
  COL_CHASSIS = BIT(2), // Collide with car chassis
  COL_WHEEL = BIT(3)    // Collide with wheel
};

public:
  spBulletWorld();
  ~spBulletWorld();

  bool InitEmptyDynamicsWorld();
  void AddNewPhyObject(spCommonObject& sp_obj);
  void UpdatePhyObjectsFromSpirit(Objects& spobj);
  void StepPhySimulation(double step_time);
  void UpdateSpiritObjectsFromPhy(Objects& spobjects);

private:
  void UpdateBulletBoxObject(spBox& source_obj, btRigidBody* dest_obj);
  void UpdateBulletVehicleObject(spVehicle& source_obj, btRigidBody* dest_obj);
  btRigidBody* CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape);
  btTransform& spPose2btTransform(const spPose& pose, double btworld_scale);
  spPose& btTransform2spPose(const btTransform& tr, double btworld_scale_inv);
  btRigidBody* CreateBulletVehicleObject(spVehicle& source_obj);
  btRigidBody* CreateBulletBoxObject(spBox& source_obj);
  void ClampObjectsToSurfaces(Objects &spobj);
  BulletWorldParams world_params_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher*	dispatcher_;
  btBroadphaseInterface*	broadphase_;
  btConstraintSolver*	solver_;
  btDiscreteDynamicsWorld* dynamics_world_;
  btDantzigSolver* solver_dantzig_;
  btSolveProjectedGaussSeidel* solver_gseidel_;
  btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
  int chassis_collides_with_;
  int wheel_collides_with_;
  int box_collides_with_;
};

#endif // SP_BULLETWORLD_H__
