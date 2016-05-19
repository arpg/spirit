#ifndef SP_BOX_H__
#define SP_BOX_H__

#include <spirit/Objects/spCommonObject.h>

class spBox : public spCommonObject {
public:
  spBox();
  ~spBox();
  void SetDimensions(const spBoxSize& dims);
  spBoxSize GetDimensions();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  void SetMass(double mass);
  double GetMass();

private:
  spBoxSize dims_;
  spPose pose_;
  spColor color_;
  double mass_;

};

#endif  //  SP_BOX_H__
