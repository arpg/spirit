#ifndef SP_WAYPOINT_H__
#define SP_WAYPOINT_H__

#include <spirit/Objects/spCommonObject.h>

class spWaypoint : public spCommonObject {
 public:
  spWaypoint(const spPose& pose, const spColor& color);
  spWaypoint(const spWaypoint&wp);
  ~spWaypoint();
  void SetPose(const spPose& pose);
  const spPose& GetPose() const;
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();
  const spLinVel GetLinearVelocity() const;

//  void SetLinearVelocityDirection(const spLinVel& linvel);

  void SetLinearVelocityNorm(double length);
  double GetLinearVelocityNorm();

 private:
  double linvelnorm_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_WAYPOINT_H__
