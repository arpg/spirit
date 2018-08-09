#ifndef SP_LINESTRIP_H__
#define SP_LINESTRIP_H__

#include <spirit/Objects/spCommonObject.h>

class spLineStrip : public spCommonObject {
 public:
  spLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color);
  spLineStrip(const spPose& pose, const spCurve& curve,int num_points, const spColor& color);
  ~spLineStrip();

  // SetPose here corresponds to the base of the curve
  void SetPose(const spPose& pose);
  const spPose& GetPose();

  void SetColor(const spColor& color);
  const spColor& GetColor();

  void SetLineStripPoints(const spPoints3d& pts);
  void SetLineStripPointsFromCurve(const spCurve& curve, int num_pts);
  spPoints3d& GetLineStripPoints();

 private:
  spColor color_;
  spPose pose_;
  std::shared_ptr<spPoints3d> points_;
};

#endif  //  SP_LINESTRIP_H__
