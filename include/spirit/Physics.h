#ifndef PHYSICS_H__
#define PHYSICS_H__

#include <spirit/Physics/spPhysicsWorld.h>
#include <spirit/Physics/spBulletWorld.h>
#include <spirit/spSettings.h>

class Physics {
 public:
  Physics();
  ~Physics();
  void Create(const spPhyEngineType phy_type);
  void AddBox(spBox& box);
  void AddSphere(spSphere& sphere);
  void AddCar(spCarParamseters& car_params);

 private:
  std::shared_ptr<spPhysicsWorld> phyworld_;
};

#endif  // PHYSICS_H__
