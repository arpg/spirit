#ifndef SP_TYPES_H__
#define SP_TYPES_H__

#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <iostream>

#warning "Comment the following line if using bullet in single precision mode"
#define BT_USE_DOUBLE_PRECISION

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <bullet/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.h>

// bullet definitions
#define USE_MOTIONSTATE 1
#define SP_PI 3.14159265359
#define SP_PI_HALF 1.57079632679
#define SP_PI_QUART 0.78539816339

// for a stable physics result use a scale of 2-10
#define WSCALE 10
#define WSCALE_INV 0.1
#define BULLET_SOLVER_NUM_ITERATIONS 10

#define BIT(x) (1<<(x))
enum BulletCollissionType{
  COL_NOTHING = 0,      // Collide with nothing
  COL_BOX = BIT(0),     // Collide with box
  COL_MESH = BIT(1),    // Collide with mesh
  COL_CHASSIS = BIT(2), // Collide with car chassis
  COL_WHEEL = BIT(3)    // Collide with wheel
};

// spirit types
typedef Eigen::Transform<double,3,Eigen::Affine> spPose;
typedef Eigen::Quaterniond spRotation;
typedef Eigen::Vector3d spTranslation;
typedef Eigen::Vector3d spBoxSize;
typedef Eigen::Vector3d spLinVel;
typedef Eigen::Vector3d spRotVel;
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
typedef Eigen::Matrix<double,2,3> spCtrlPts2ord_2dof;
// rows = [x,y,z,q1,q2,q3,q4,x_d,y_d,z_d,roll_d,pitch_d,yaw_d]
// cols = [bezP1x,bezP1y,bezP2x,bezP2y,bezP3x,bezP3y,bezP4x,bezP4y]
//typedef Eigen::Matrix<double,6,6> spPlannerJacobian;
typedef Eigen::Matrix<double,6,4> spPlannerJacobian;
// spStateVec means [x,y,yaw,x_d,y_d,yaw_d]
typedef Eigen::Array<double,6,1> spStateVec;
typedef std::chrono::high_resolution_clock::time_point spTimestamp;
typedef Eigen::Matrix4d spMat4x4;

#define SPERROREXIT(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl; std::exit(EXIT_FAILURE)
#define SPERROR(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl

enum spPhysolver{MLCP_DANTZIG,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum spPhyEngineType{PHY_NONE,PHY_BULLET};
enum spObjectType{BOX,VEHICLE_AWSD,VEHICLE_AWD,VEHICLE_GENERAL,VEHICLE_RWD,WHEEL,WAYPOINT,LINESTRIP};

struct spVehicleConstructionInfo{
  spObjectType vehicle_type;
  spPose pose;
  spTranslation cog; // Center Of Gravity in chassis frame
  std::vector<spTranslation> wheels_anchor;
  double chassis_mass;
  spColor color;
  spBoxSize chassis_size;
  double chassis_friction;
  // wheel info
  double wheel_friction;  // unit less
  double wheel_rollingfriction;  // unit less
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
    vehicle_type = spObjectType::VEHICLE_GENERAL;
    pose = spPose::Identity();
    color = spColor(0,1,0);
    cog = spTranslation(0,0,0);
    chassis_mass = 0;
    chassis_size = spBoxSize(0,0,0);
    chassis_friction = 0;
    wheel_friction = 0;
    wheel_rollingfriction = 0;
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
  spVehicleConstructionInfo(const spVehicleConstructionInfo& v)
    : vehicle_type(v.vehicle_type),
      pose(v.pose),
      cog(v.cog),
      chassis_mass(v.chassis_mass),
      color(v.color),
      chassis_size(v.chassis_size),
      chassis_friction(v.chassis_friction),
      wheel_friction(v.wheel_friction),
      wheel_rollingfriction(v.wheel_rollingfriction),
      wheel_width(v.wheel_width),
      wheel_radius(v.wheel_radius),
      susp_damping(v.susp_damping),
      susp_stiffness(v.susp_stiffness),
      susp_preloading_spacer(v.susp_preloading_spacer),
      susp_lower_limit(v.susp_lower_limit),
      susp_upper_limit(v.susp_upper_limit),
      steering_servo_lower_limit(v.steering_servo_lower_limit),
      steering_servo_upper_limit(v.steering_servo_upper_limit),
      wheel_mass(v.wheel_mass) {
    for(int ii=0;ii<v.wheels_anchor.size();ii++) {
      wheels_anchor.push_back(v.wheels_anchor[ii]);
    }
  }
};

class spCurve {
 public:

  spCurve(int curve_order, int curve_dof): curve_order_(curve_order), curve_dof_(curve_dof) {
    perturbation_value_ = 0;
    perturbation_dim_ = 0;
  }

  ~spCurve() {}

  void PerturbControlPoint(unsigned int dim2perturb, double delta) {
    if(dim2perturb>((curve_dof_*(curve_order_+1)-1))) {
      SPERROREXIT("Requested Dim doesn't exist");
    }
    perturbation_value_ = delta;
    perturbation_dim_ = dim2perturb;
//    std::cout << "dim is " << perturbation_dim_ << "and data is " << ctrl_pts_.data()[perturbation_dim_] << std::endl;
    ctrl_pts_.data()[perturbation_dim_] += perturbation_value_;
  }

  void RemoveLastPerturbation() {
    ctrl_pts_.data()[perturbation_dim_] -= perturbation_value_;
    perturbation_value_ = 0;
  }

  void SetBezierControlPoints(const Eigen::MatrixXd& pts) {
    if((pts.rows() != curve_dof_)||(pts.cols() != curve_order_+1)) {
      SPERROREXIT("Matrix dimension mismatch.");
    }
    ctrl_pts_ = pts;
  }

  void SetHermiteControlPoints(const Eigen::MatrixXd& pts) {
    if((pts.rows() != curve_dof_)||(pts.cols() != curve_order_+1)) {
      std::cout << "Matrix has " << pts.rows() << " rows and " << pts.cols() << " colums." << std::endl;
      std::cout << "curve dof is " << curve_dof_ << " curve order is " << curve_order_ << std::endl;
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
    if(curve_order_ == 3) {
      point = (pow((1 - t), 3) * ctrl_pts_.col(0)) +
          (3 * pow((1 - t), 2) * t * ctrl_pts_.col(1)) +
          (3 * (1 - t) * pow(t, 2) * ctrl_pts_.col(2)) +
          (pow(t, 3) * ctrl_pts_.col(3));
    } else if(curve_order_ == 2) {
      point = (pow((1 - t), 2) * ctrl_pts_.col(0)) +
          (2 * (1 - t) * t * ctrl_pts_.col(1)) +
          (pow(t, 2) * ctrl_pts_.col(2));
    } else {
      SPERROREXIT("Not Implemented.");
    }
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

  // first 3d dimentions (x,y,z) are used for visualization purposes
  void GetPoints3d(spPoints3d& pts_vec/*, int num_mid_pts*/) {
//    int num_pts = num_mid_pts - 1;
    if(pts_vec.size()<1) {
      SPERROREXIT("array size must be greater than zero !");
    }
    int num_pts = pts_vec.size() - 1;
    for (int t = 0; t <= num_pts; t++) {
      spPointXd point(curve_dof_);
      this->GetPoint(point, t * (1.0 / num_pts));
//      pts_vec.push_back(point.head(3));
      pts_vec[t] = point.head(3);
    }
  }

  // given number of circles find that many curvatures along curve and return max
  double GetMaxCurvature(unsigned int num_circles) {
    spPointsXd points;
    GetPointsXd(points,2*num_circles+1);

  }

private:
 // Control points are in local coordinate of the curve
 Eigen::MatrixXd ctrl_pts_;
 int curve_dof_;
 int curve_order_;
 double perturbation_value_;
 int perturbation_dim_;
};
#endif  // SP_TYPES_H__
