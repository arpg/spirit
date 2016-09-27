#ifndef SP_LOCALPLANNER_H__
#define SP_LOCALPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>

class spLocalPlannerInterface {
public:
  virtual void RegisterPhysicsWorld() = 0;
  virtual void PlanTrajectory(spPose start, spPose goal) = 0;
};

#endif // SP_LOCALPLANNER_H__
