#ifndef OBJECTS_H__
#define OBJECTS_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects/spBox.h>
#include <spirit/Objects/spVehicle.h>
#include <spirit/Objects/spAWSDCar.h>
#include <spirit/Objects/spBike.h>
#include <spirit/Objects/spWaypoint.h>
#include <spirit/Objects/spLineStrip.h>
#include <spirit/Objects/spMesh.h>
#include <osg/Node>
#include <osg/ref_ptr>


typedef std::list<std::shared_ptr<spCommonObject>>::iterator spObjectHandle;

class Objects {

  struct BulletWorldParams{
    btVector3 worldMin;
    btVector3 worldMax;
    spPhysolver solver;
  };


public:
  Objects();
  Objects(const Objects& obj);
  ~Objects();
  spObjectHandle CreateBox(const spPose& pose, const spBoxSize& size, double mass,const spColor& color);
  spObjectHandle CreateWaypoint(const spPose& pose, const spColor& color);
  spObjectHandle CreateVehicle(const spVehicleConstructionInfo& vehicle_info);
  spObjectHandle CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color);
  spObjectHandle CreateLineStrip(const spPose& pose, const spCurve& curve, int num_pts, const spColor& color);
  spObjectHandle CreateMesh(const osg::ref_ptr<osg::Node>& meshnode);
  void StepPhySimulation(double step_time);
  void RemoveObj(spObjectHandle& obj_handle);
  int GetNumOfObjects();
  spCommonObject& GetObject(spObjectHandle& obj_handle);
  spObjectHandle GetListBegin();
  spObjectHandle GetListEnd();

private:
  spPose& Transform2spPose(const btTransform& tr, double btworld_scale_inv);
  btTransform& spPose2btTransform(const spPose& pose, double btworld_scale);
  btRigidBody* CreateRigidBody(double mass, const btTransform& tr, btCollisionShape* shape);
  std::list<std::shared_ptr<spCommonObject>> objects_;
  void InitEmptyDynamicsWorld();
  BulletWorldParams world_params_;
  std::shared_ptr<btDefaultCollisionConfiguration> collisionConfiguration_;
  std::shared_ptr<btCollisionDispatcher>	dispatcher_;
  std::shared_ptr<btBroadphaseInterface>	broadphase_;
  std::shared_ptr<btConstraintSolver>	solver_;
  std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;
  std::shared_ptr<btDantzigSolver> solver_dantzig_;
  std::shared_ptr<btSolveProjectedGaussSeidel> solver_gseidel_;
  // when ever we remove a object we are gonna point its handle to NULL_HANDLE
  const spObjectHandle NULL_HANDLE;
//  btAlignedObjectArray<btCollisionShape*>	collisionShapes_;
};

#endif  //  OBJECTS_H__
