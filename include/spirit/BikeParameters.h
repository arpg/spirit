#ifndef BikeParameters_H__
#define BikeParameters_H__

#include <spirit/Types/spTypes.h>
#include <eigen3/Eigen/Eigen>
//#include <spirit/spirit.h>

class BikeParams {
   public:
    spVehicleConstructionInfo bike_param;
    BikeParams(){
        bike_param.vehicle_type = spObjectType::VEHICLE_BIKE;
        bike_param.pose = spPose::Identity();
        bike_param.pose.translate(spTranslation(0,0,0.0));
        bike_param.wheels_anchor.push_back(spTranslation(0, -0.27, 0));
        bike_param.wheels_anchor.push_back(spTranslation(0, 0.27, 0));
        bike_param.chassis_size = spBoxSize(0.1, 0.42, 0.05);
        bike_param.cog = spTranslation(0, 0, 0);
        bike_param.chassis_friction = 0;
        bike_param.wheel_rollingfriction = 0.1;
        bike_param.wheel_friction = 0.5;
        bike_param.wheel_width = 0.04;
        bike_param.wheel_radius = 0.05;
        bike_param.susp_damping = 0;
        bike_param.susp_stiffness = 10;
        bike_param.susp_preloading_spacer = 0.1;
        bike_param.susp_upper_limit = 0.013;
        bike_param.susp_lower_limit = -0.028;
        bike_param.wheel_mass = 0.1;
        bike_param.chassis_mass = 5;
        bike_param.engine_torque = 100;
        bike_param.steering_servo_lower_limit = -SP_PI / 4;
        bike_param.steering_servo_upper_limit = SP_PI / 4;
        bike_param.steering_servo_max_velocity = 100;
        bike_param.steering_servo_torque = 100;

    }
    ~BikeParams(){}

};

#endif //BikeParameters_H__
