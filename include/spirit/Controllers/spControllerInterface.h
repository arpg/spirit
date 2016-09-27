#ifndef SP_CONTROLLERINTERFACE_H__
#define SP_CONTROLLERINTERFACE_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>

class spControllerInterface {
public:
  virtual void docontrollstuff() = 0;
};

#endif // SP_CONTROLLERINTERFACE_H__
