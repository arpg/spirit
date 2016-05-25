#ifndef SP_BULLETWORLD_H__
#define SP_BULLETWORLD_H__

#include <iostream>
#include <spirit/Physics/spPhysicsWorld.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.h>

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
  void AddBox(spBox& box);
  void AddSphere(spSphere& sphere);
  void AddCar(spCarParamseters& car_params);
  void UpdatePhyObjects(Objects& spobj);

private:
  int object_counter;
  void AddRigidBodyToWorld(btCollisionShape* shape);
  BulletWorldParams world_params_;
  btCollisionShape* def_groundShape_;
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
