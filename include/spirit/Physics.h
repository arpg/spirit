#ifndef PHYSICS_H__
#define PHYSICS_H__

#include <spirit/Physics/spPhysicsWorld.h>
#include <spirit/Physics/spBulletWorld.h>
#include <spirit/spSettings.h>
#include <spirit/spGeneralTools.h>

class Physics {
 public:
  Physics();
  ~Physics();
  void Create(const spPhyEngineType phy_type);
  void AddObject(spCommonObject& obj);
  void Iterate(Objects& objects, double sim_sec);

 private:
  std::shared_ptr<spPhysicsWorld> phyworld_;
};

#endif  // PHYSICS_H__
