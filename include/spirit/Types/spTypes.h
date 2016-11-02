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
typedef Eigen::Vector3d spPoint3d;
typedef Eigen::VectorXd spPointXd;
typedef Eigen::Array<double,6,1> spVelocity;
typedef std::vector<spPoint3d,Eigen::aligned_allocator<spPoint3d>> spPoints3d;
typedef std::vector<spPointXd,Eigen::aligned_allocator<spPointXd>> spPointsXd;
typedef Eigen::Matrix3d spInertiaTensor;
typedef Eigen::Vector3d spCubeInertiaTensor;
// ctrlpts for bezier curve mean [P0,P1,P2,P3]
// ctrlpts for Hermite curve mean [P0,D0,P3,D3]
typedef Eigen::Matrix<double,3,4> spCtrlPts3ord_3dof;
typedef Eigen::Matrix<double,2,4> spCtrlPts3ord_2dof;
typedef Eigen::Matrix<double,14,8> spPlannerJacob;
// spStateVec means [x,y,z,q1,q2,q3,q4,x_d,y_d,z_d,q1_d,q2_d,q3_d,q4_d]
typedef Eigen::Array<double,13,1> spStateVec;
typedef std::chrono::high_resolution_clock::time_point spTimestamp;
typedef Eigen::Matrix4d spMat4x4;

#define SPERROREXIT(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl; std::exit(EXIT_FAILURE)
#define SPERROR  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl

// bullet definitions
#define USE_MOTIONSTATE 1
#define SP_PI 3.14159265359
#define SP_PI_HALF 1.57079632679
#define SP_PI_QUART 0.78539816339

enum spPhysolver{MLCP_DANTZIG,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,VEHICLE,WHEEL,WAYPOINT,LINESTRIP};
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

class spCurve {
 public:

  spCurve(int curve_order, int curve_dof): curve_order_(curve_order), curve_dof_(curve_dof) {
  }

  ~spCurve() {}

  void SetBezierControlPoints(const Eigen::MatrixXd& pts) {
    if((pts.rows() != curve_dof_)||(pts.cols() != curve_order_+1)) {
      SPERROREXIT("Matrix dimension mismatch.");
    }
    ctrl_pts_ = pts;
  }

  void SetHermiteControlPoints(const Eigen::MatrixXd& pts) {
    if((pts.rows() != curve_dof_)||(pts.cols() != curve_order_+1)) {
      SPERROREXIT("Wrong matrix dimension requested.");
    }
    if(pts.cols() == 4) {
      // hermite to bezier conversion matrix
      spMat4x4 conv;
      conv << 1, 0, 0, 0,
          1, 1.0f/3, 0, 0,
          0, 0, 1, -1.0f/3,
          0, 0, 1, 0;
      ctrl_pts_ =  pts*conv.transpose();
    } else {
      SPERROREXIT("Requested Order of curve has not been implemented");
    }
  }

  const Eigen::MatrixXd& GetBezierControlPoints() {
    return ctrl_pts_;
  }

  void GetPoint(Eigen::VectorXd& point, double t) {
    if(point.rows() != curve_dof_) {
      SPERROREXIT("Wrong point dimention requested.");
    }

    point = (pow((1 - t), 3) * ctrl_pts_.col(0)) +
        (3 * pow((1 - t), 2) * t * ctrl_pts_.col(1)) +
        (3 * (1 - t) * pow(t, 2) * ctrl_pts_.col(2)) +
        (pow(t, 3) * ctrl_pts_.col(3));
    // calc global coordinates of the point
  //  point = pose_ * point;
  }

  void GetPointsXd(spPointsXd& pts_vec, int num_mid_pts) {
    int num_pts = num_mid_pts - 1;
    for (int t = 0; t <= num_pts; t++) {
      spPointXd point(curve_dof_);
      this->GetPoint(point, t * (1.0 / num_pts));
  //    std::shared_ptr<spPoint> point_ptr = std::make_shared<spPoint>(point);
      pts_vec.push_back(point);
    }
  }

  // firs 3d dimentions (x,y,z) are used for visualization purposes
  void GetPoints3d(spPoints3d& pts_vec, int num_mid_pts) {
    int num_pts = num_mid_pts - 1;
    for (int t = 0; t <= num_pts; t++) {
      spPointXd point(curve_dof_);
      this->GetPoint(point, t * (1.0 / num_pts));
      pts_vec.push_back(point.head(3));
    }
  }

private:
 // Control points are in local coordinate of the curve
 Eigen::MatrixXd ctrl_pts_;
 int curve_dof_;
 int curve_order_;
};
#endif  // SP_TYPES_H__