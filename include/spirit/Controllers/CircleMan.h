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

    double dist = std::sqrt(std::pow(vec[0],2)+std::pow(vec[1],2));

    // Rotational vector error
    vec[3] = 0;
    vec[4] = 0;
    double chi = state.pose.rotation().eulerAngles(0,1,2)[2];
    double chi_mod = std::fmod(chi+SP_PI,2*SP_PI);
    if(chi_mod<0){
      chi_mod += 2*SP_PI;
    }
    double theta_mod = std::fmod(theta+SP_PI,2*SP_PI);
    if(theta_mod<0){
      theta_mod += 2*SP_PI;
    }
    double diff = chi_mod-theta_mod;
    if(diff>SP_PI){
      diff -= 2*SP_PI;
    } else if (diff<-SP_PI){
      diff += 2*SP_PI;
    }
    vec[5] = diff;

//    vec[5] *= 3*(0.3-std::min(dist,0.3))/0.3;
    vec[5] *= 1.3;

    // Linear velocity error
    vec[6] = state.linvel[0] + tan_vel_*std::sin(theta);
    vec[7] = state.linvel[1] - tan_vel_*std::cos(theta);
    vec[8] = 0;

    vec[6] *= 0.4;
    vec[7] *= 0.4;

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
