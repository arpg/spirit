#ifndef SP_LINE_H__
#define SP_LINE_H__

#include <spirit/Objects/spCommonObject.h>

class spLine : public spCommonObject {
 public:
  spLine();
  ~spLine();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();

//  void SetDimensions(const spLineSize& dims);
//  spLineSize GetDimensions();

 private:
  spPose pose_;
  spColor color_;
};

#endif  //  SP_LINE_H__
