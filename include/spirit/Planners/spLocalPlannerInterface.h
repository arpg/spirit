#ifndef SP_LOCALPLANNER_H__
#define SP_LOCALPLANNER_H__

#include <spirit/Types/spTypes.h>
#include <spirit/Objects.h>

class spLocalPlannerInterface {
public:
  virtual const spBezierCtrlPoints& GetPlan(const spPose& start, const spPose& goal) = 0;
};

#endif // SP_LOCALPLANNER_H__
