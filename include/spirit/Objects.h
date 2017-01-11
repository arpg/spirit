#ifndef OBJECTS_H__
#define OBJECTS_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spBox.h>
#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spAWSDCar.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>

class Objects {

  struct BulletWorldParams{
    btVector3 worldMin;
    btVector3 worldMax;
    spPhysolver solver;
  };

public:
  Objects();
  ~Objects();
  int CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color);
  int CreateWaypoint(const spPose& pose, const spColor& color);
  int CreateVehicle(const spVehicleConstructionInfo& vehicle_info);
  int CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color);
  void StepPhySimulation(double step_time);
  void RemoveObj(int obj_index);
  int GetNumOfObjects();
  spCommonObject& GetObject(int obj_index);

private:
  std::vector<std::shared_ptr<spCommonObject>> objects_;

  void InitEmptyDynamicsWorld();
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

#endif  //  OBJECTS_H__
