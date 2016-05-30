#ifndef SP_PHYSICSWORLD_H__
#define SP_PHYSICSWORLD_H__

#include <spirit/spGeneralTools.h>
#include <spirit/Objects.h>

class spPhysicsWorld {
public:
  virtual bool InitEmptyDynamicsWorld() = 0;
  virtual void AddNewPhyObject(spCommonObject& sp_object) = 0;
  virtual void UpdatePhyObjectsFromSpirit(Objects& sp_objects) = 0;
  virtual void UpdateSpiritObjectsFromPhy(Objects& sp_objects) = 0;
  virtual void StepPhySimulation(double step_time) = 0;
};

#endif // SP_PHYSICSWORLD_H__
