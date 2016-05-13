#ifndef SP_PHYSICSWORLD_H__
#define SP_PHYSICSWORLD_H__

#include <spirit/GeneralTools.h>

class spPhysicsWorld {
public:
  virtual bool InitEmptyDynamicsWorld() = 0;
  virtual void AddRigidBody(spRigidBodyType& rbtype) = 0;
};

#endif // SP_PHYSICSWORLD_H__
