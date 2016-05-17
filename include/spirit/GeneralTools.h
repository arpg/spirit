#ifndef GENERALTOOLS_H__
#define GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <Eigen/Eigen>

// spirit types
#define spPose Eigen::Transform<double,3,Eigen::Affine>
#define spBoxSize Eigen::Vector3d
#define spVector3d Eigen::Vector3d
#define spColor Eigen::Vector3d
#define spInertiaTensor Eigen::Matrix3d
#define spCubeInertiaTensor Eigen::Vector3d

// bullet definitions
#define USE_MOTIONSTATE 1

enum spPhysolver{MLCP_DANTZING,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};

struct spBox{
  spBoxSize dims;
  double mass;
  spPose pose;
  spColor color;
};

struct spSphere{
  double  radius;
  double mass;
  spPose pose;
  spColor color;
};
struct spCarParamseters{
  spPose pose;
  float wheel_friction;
};


class GeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
};

#endif  // GENSERALTOOLS_H__
