#ifndef SP_COMMONOBJECT_H__
#define SP_COMMONOBJECT_H__

#include <spirit/spGeneralTools.h>

/// Any spirit object should only have instances of spirit classes. like
/// spphysics, spgui
class spCommonObject {
 public:
  virtual void SetPose(const spPose& pose) = 0;
  virtual const spPose& GetPose() = 0;
  virtual void SetColor(const spColor& color) = 0;
  virtual void SetMass(double mass) = 0;
};

#endif  //  SP_COMMONOBJECT_H__
