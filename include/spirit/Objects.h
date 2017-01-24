#ifndef OBJECTS_H__
#define OBJECTS_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spBox.h>
#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spAWSDCar.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>


typedef std::list<std::shared_ptr<spCommonObject>>::iterator spObjectHandle;

class Objects {

  struct BulletWorldParams{
    btVector3 worldMin;
    btVector3 worldMax;
    spPhysolver solver;
  };


public:
  Objects();
  ~Objects();
  spObjectHandle CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color);
  spObjectHandle CreateWaypoint(const spPose& pose, const spColor& color);
  spObjectHandle CreateVehicle(const spVehicleConstructionInfo& vehicle_info);
  spObjectHandle CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color);
  void StepPhySimulation(double step_time);
  void RemoveObj(spObjectHandle& obj_handle);
  int GetNumOfObjects();
  spCommonObject& GetObject(spObjectHandle& obj_handle);
  spObjectHandle GetListBegin();
  spObjectHandle GetListEnd();

int boxtest();

private:

spPose& btTransform2spPose(const btTransform& tr, double btworld_scale_inv);
btTransform& spPose2btTransform(const spPose& pose, double btworld_scale);
btRigidBody* CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape);

  std::list<std::shared_ptr<spCommonObject>> objects_;
  void InitEmptyDynamicsWorld();
  BulletWorldParams world_params_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher*	dispatcher_;
  btBroadphaseInterface*	broadphase_;
  btConstraintSolver*	solver_;
  btDiscreteDynamicsWorld* dynamics_world_;
  btDantzigSolver* solver_dantzig_;
  btSolveProjectedGaussSeidel* solver_gseidel_;
  // when ever we remove a object we are gonna point its handle to NULL_HANDLE
  const spObjectHandle NULL_HANDLE;
//  btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
};

#endif  //  OBJECTS_H__
