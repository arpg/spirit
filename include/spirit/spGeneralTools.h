#ifndef SP_GENERALTOOLS_H__
#define SP_GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

// spirit types
#define spPose Eigen::Transform<double,3,Eigen::Affine>
#define spRotation Eigen::Quaterniond
#define spTranslation Eigen::Vector3d
#define spBoxSize Eigen::Vector3d
#define spCylinderSize Eigen::Vector3d
#define spMeshSize Eigen::Vector2d
#define spColor Eigen::Vector3d
#define spInertiaTensor Eigen::Matrix3d
#define spCubeInertiaTensor Eigen::Vector3d

// bullet definitions
#define USE_MOTIONSTATE 1
#define SP_PI 3.14159265359
#define SP_PI_HALF 1.57079632679
#define SP_PI_QUART 0.78539816339

enum spPhysolver{MLCP_DANTZIG,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,VEHICLE,WHEEL,WAYPOINT};
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

class spGeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
};

#endif  // SP_GENSERALTOOLS_H__
