#ifndef SP_PHYSICSWORLD_H__
#define SP_PHYSICSWORLD_H__

#include <spirit/GeneralTools.h>

class spPhysicsWorld {
public:
  virtual bool InitEmptyDynamicsWorld() = 0;
  virtual int AddBox(spBox& box) = 0;
  virtual int AddSphere(spSphere& sphere) = 0;
  virtual int AddCar(spCarParamseters& car_params) = 0;
};

#endif // SP_PHYSICSWORLD_H__
