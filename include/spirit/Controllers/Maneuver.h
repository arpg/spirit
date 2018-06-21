#ifndef MANEUVER_H__
#define MANEUVER_H__

#include <spirit/Types/spTypes.h>

class Maneuver {
public:
  virtual spState GetClosestPoint(const spState& state) const = 0;
  virtual spStateVec GetStateError(const spState& state) const = 0;
  spManeuverType GetManeuverType() {
    return man_type_;
  }

protected:
  spManeuverType man_type_;
};

#endif // MANEUVER_H__
