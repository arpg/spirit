#ifndef SP_BOX_H__
#define SP_BOX_H__

#include <spirit/Objects/spCommonObject.h>

class spBox : public spCommonObject {
 public:
  spBox();
  ~spBox();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();

  void SetMass(double mass);
  double GetMass();

  void SetDimensions(const spBoxSize& dims);
  spBoxSize GetDimensions();

 private:
  spBoxSize dims_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_BOX_H__
