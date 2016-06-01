#ifndef SP_GENERALTOOLS_H__
#define SP_GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <Eigen/Eigen>
#include <iostream>

// spirit types
#define spPose Eigen::Transform<double,3,Eigen::Affine>
#define spRotation Eigen::Quaternion<double>
#define spTranslation Eigen::Vector3d
#define spBoxSize Eigen::Vector3d
#define spMeshSize Eigen::Vector2d
#define spColor Eigen::Vector3d
#define spInertiaTensor Eigen::Matrix3d
#define spCubeInertiaTensor Eigen::Vector3d

// bullet definitions
#define USE_MOTIONSTATE 1

enum spPhysolver{MLCP_DANTZING,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,SPHERE,MESH};

struct spSphere{
  double  radius;
  double mass;
  spPose pose;
  spColor color;
};

class spGeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
};

#endif  // SP_GENSERALTOOLS_H__
