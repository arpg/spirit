#ifndef SP_TYPES_H__
#define SP_TYPES_H__

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

// spirit types
typedef Eigen::Transform<double,3,Eigen::Affine> spPose;
typedef Eigen::Quaterniond spRotation;
typedef Eigen::Vector3d spTranslation;
typedef Eigen::Vector3d spBoxSize;
typedef Eigen::Vector3d spCylinderSize;
typedef Eigen::Vector2d spMeshSize;
typedef Eigen::Vector3d spColor;
typedef Eigen::Vector3d spPoint;
typedef std::vector<std::shared_ptr<Eigen::Vector3d>> spPoints;
typedef Eigen::Matrix3d spInertiaTensor;
typedef Eigen::Vector3d spCubeInertiaTensor;
typedef Eigen::Matrix<double, 3, 4> spBezierCtrlPoints;
typedef std::chrono::high_resolution_clock::time_point spTimestamp;

// bullet definitions
#define USE_MOTIONSTATE 1
#define SP_PI 3.14159265359
#define SP_PI_HALF 1.57079632679
#define SP_PI_QUART 0.78539816339

enum spPhysolver{MLCP_DANTZIG,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,VEHICLE,WHEEL,WAYPOINT,BEZIER_CURVE};
enum spVehicleConfig{AWSD};

struct spVehicleConstructionInfo{
  spVehicleConfig vehicle_type;
  spPose pose;
  spTranslation cog; // Center Of Gravity in chassis frame
  std::vector<spTranslation> wheels_anchor;
  double chassis_mass;
  spColor color;
  spBoxSize chassis_size;
  // wheel info
  double wheel_friction;  // unit less
  double wheel_width;     // in meters
  double wheel_radius;    // in meters
  double susp_damping;
  double susp_stiffness;
  double susp_preloading_spacer; //in meters
  double susp_lower_limit;
  double susp_upper_limit;
  double steering_servo_lower_limit;
  double steering_servo_upper_limit;
  double wheel_mass;

  spVehicleConstructionInfo(){
    vehicle_type = spVehicleConfig::AWSD;
    pose = spPose::Identity();
    color = spColor(0,1,0);
    cog = spTranslation(0,0,0);
    chassis_mass = 0;
    chassis_size = spBoxSize(0,0,0);
    wheel_friction = 0;
    wheel_width = 0;
    wheel_radius = 0;
    susp_damping = 0;
    susp_stiffness = 0;
    susp_preloading_spacer = 0;
    susp_lower_limit = 0;
    susp_upper_limit = 0;
    steering_servo_lower_limit = 0;
    steering_servo_upper_limit = 0;
    wheel_mass = 0;
  }
};

#endif  // SP_TYPES_H__
