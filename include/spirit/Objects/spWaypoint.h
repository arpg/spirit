#ifndef SP_WAYPOINT_H__
#define SP_WAYPOINT_H__

#include <spirit/Objects/spCommonObject.h>

class spWaypoint : public spCommonObject {
 public:
  spWaypoint();
  spWaypoint(const spWaypoint&wp);
  ~spWaypoint();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();

  void SetLength(double length);
  double GetLength();

 private:
  double length_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_WAYPOINT_H__
