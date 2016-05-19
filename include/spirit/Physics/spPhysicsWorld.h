#ifndef SP_PHYSICSWORLD_H__
#define SP_PHYSICSWORLD_H__

#include <spirit/spGeneralTools.h>
#include <spirit/Objects/spBox.h>

class spPhysicsWorld {
public:
  virtual bool InitEmptyDynamicsWorld() = 0;
  virtual void AddBox(spBox& box) = 0;
  virtual void AddSphere(spSphere& sphere) = 0;
  virtual void AddCar(spCarParamseters& car_params) = 0;
};

#endif // SP_PHYSICSWORLD_H__
