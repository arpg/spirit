#ifndef SP_BULLETWORLD_H__
#define SP_BULLETWORLD_H__

#include <iostream>
#include <spirit/Physics/spPhysicsWorld.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.h>

// for a stable physics result use a scale of 2-10
#define WSCALE 2
#define WSCALE_INV 0.5

struct BulletWorldParams{
  btVector3 worldMin;
  btVector3 worldMax;
  spPhysolver solver;
};

class spBulletWorld: public spPhysicsWorld {
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
  btTransform spPose2btTransform(const spPose& pose, double btworld_scale);
  spPose btTransform2spPose(const btTransform& tr, double btworld_scale_inv);
  btRigidBody* CreateBulletVehicleObject(spVehicle& source_obj);
  btRigidBody* CreateBulletBoxObject(spBox& source_obj);

  BulletWorldParams world_params_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher*	dispatcher_;
  btBroadphaseInterface*	broadphase_;
  btConstraintSolver*	solver_;
  btDiscreteDynamicsWorld* dynamics_world_;
  btDantzigSolver* solver_dantzig_;
  btSolveProjectedGaussSeidel* solver_gseidel_;

  btAlignedObjectArray<btCollisionShape*>	collisionShapes_;

};

#endif // SP_BULLETWORLD_H__
