#ifndef BikeParameters_H__
#define BikeParameters_H__

#include <spirit/Types/spTypes.h>
#include <eigen3/Eigen/Eigen>
//#include <spirit/spirit.h>

class BikeParams {
   public:
    spVehicleConstructionInfo car_param;
    BikeParams(){
        car_param.vehicle_type = spObjectType::VEHICLE_BIKE;
        car_param.pose = spPose::Identity();
        car_param.pose.translate(spTranslation(0,0,0));
        car_param.wheels_anchor.push_back(spTranslation(0, -0.27, -0.003));
        car_param.wheels_anchor.push_back(spTranslation(0, 0.27, -0.003));
        car_param.chassis_size = spBoxSize(0.1, 0.42, 0.05);
        car_param.cog = spTranslation(0, 0, 0);
        car_param.chassis_friction = 0;
        car_param.wheel_rollingfriction = 0.1;
        car_param.wheel_friction = 0.5;
        car_param.wheel_width = 0.04;
        car_param.wheel_radius = 0.05;//0.057;
        car_param.susp_damping = 0;
        car_param.susp_stiffness = 10;
        car_param.susp_preloading_spacer = 0.1;
        car_param.susp_upper_limit = 0.013;
        car_param.susp_lower_limit = -0.028;
        car_param.wheel_mass = 0.1;
        car_param.chassis_mass = 5;
        car_param.engine_torque = 100;
        car_param.steering_servo_lower_limit = -SP_PI / 4;
        car_param.steering_servo_upper_limit = SP_PI / 4;
        car_param.steering_servo_max_velocity = 100;
        car_param.steering_servo_torque = 100;

    }
    ~BikeParams(){}

};

#endif //BikeParameters_H__
