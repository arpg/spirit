#ifndef CIRCLEMAN_H__
#define CIRCLEMAN_H__

#include <spirit/Controllers/Maneuver.h>

class CircleMan : public Maneuver{
public:
  CircleMan(double radius, double tan_vel):radius_(radius),tan_vel_(tan_vel){
    man_type_ = spManeuverType::CIRCLE;
  }

  CircleMan(const CircleMan& init):radius_(init.radius_),tan_vel_(init.tan_vel_){
  }

  spState GetClosestPoint(const spState& state) const {
    SPERROREXIT("Not Implemented !");
    return state;
  }

  spStateVec GetStateError(const spState& state) const {
    double theta = std::atan2(state.pose.translation()[1],state.pose.translation()[0]);
    spStateVec vec;
    vec[0] = state.pose.translation()[0]-radius_*std::cos(theta);
    vec[1] = state.pose.translation()[1]-radius_*std::sin(theta);
    vec[2] = 0;

    // Rotational vector error
    vec[3] = 0;
    vec[4] = 0;
    vec[5] = 0;

    // Linear velocity error
    vec[6] = state.linvel[0] + tan_vel_*std::sin(theta);
    vec[7] = state.linvel[1] - tan_vel_*std::cos(theta);
    vec[8] = 0;
    vec[6] *= 0.5;
    vec[7] *= 0.5;
//    std::cout << "linvel0 is " << state.linvel[0] << "  --  " << -tan_vel_*std::sin(theta) << std::endl;
//    std::cout << "linvel1 is " << state.linvel[1] << "  --  " << tan_vel_*std::cos(theta) << std::endl;

    // Rotational velocity error
    vec[9] = 0;
    vec[10] = 0;
    vec[11] = 0;

    return vec;
  }

  double radius_;
  double tan_vel_;

};

#endif // CIRCLEMAN_H__
