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
  const spLinVel GetLinearVelocityInWorld() const;
  void SetLinearVelocityDirection(const spLinVel& linvel_dir);
  void SetLinearVelocityNorm(double length);
  double GetLinearVelocityNorm();
  void SetRotVel(const spRotVel& rotvel);
  const spRotVel& GetRotVel();

 private:
  double linvelnorm_;
  spLinVel linveldir_;
  spRotVel rotvel_;
  spPose pose_;
  spColor color_;
};

#endif  //  SP_WAYPOINT_H__
