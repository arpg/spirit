#ifndef SP_GLOBALPLANNER_H__
#define SP_GLOBALPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>

class spGlobalPlannerInterface {
public:
  virtual void RegisterPhysicsWorld() = 0;
  virtual void PlanTrajectory(spPose start, spPose goal) = 0;
};

#endif // SP_GLOBALPLANNER_H__
