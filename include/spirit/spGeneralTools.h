#ifndef SP_GENERALTOOLS_H__
#define SP_GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <Eigen/Eigen>
#include <iostream>

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

enum spPhysolver{MLCP_DANTZING,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,CAR,WHEEL};

class spGeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
};

#endif  // SP_GENSERALTOOLS_H__
