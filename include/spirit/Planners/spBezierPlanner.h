
#include <spirit/Planners/spLocalPlannerInterface.h>

class spBezierPlanner : public spLocalPlannerInterface {
public:
  void RegisterPhysicsWorld();
  void PlanTrajectory(spPose start, spPose goal);
};
