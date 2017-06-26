#ifndef SP_TYPES_H__
#define SP_TYPES_H__

#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <iostream>

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

// error print formatting
#define SPERROREXIT(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl; std::exit(EXIT_FAILURE)
#define SPERROR(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl
#define SPERROR(X)  std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << X << std::endl

// for a stable physics result use a scale of 2-10
#define WSCALE 10
#define WSCALE_INV 0.1

#define BULLET_SOLVER_NUM_ITERATIONS 5

// Enable this line for using central differencing instead of forward diff
//#define SOLVER_USE_CENTRAL_DIFF

// recommended step sizes
// Forward diff step = 0.09
// Central diff step = 0.05
#define FINITE_DIFF_EPSILON 0.09

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
typedef Eigen::Vector4d spWheelSpeedVec;
typedef Eigen::Vector3d spBoxSize;
typedef Eigen::Vector3d spLinVel;
typedef Eigen::Vector3d spVector3;
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
typedef Eigen::Matrix<double,8,4> spPlannerJacobian;
typedef Eigen::Array<double,12,1> spStateVec;
typedef Eigen::Array<double,8,1> spResidualVec;
class spState;
typedef std::vector<std::shared_ptr<spState>> spStateSeries;

class spState {
public:
  spState() : pose(spPose::Identity()),
    linvel(spLinVel(0,0,0)),
    rotvel(spRotVel(0,0,0)) {}

  ~spState(){}

  spState(const spState& state) {
    pose = state.pose;
    linvel = state.linvel;
    rotvel = state.rotvel;
    wheel_speeds = state.wheel_speeds;
    front_steering = state.front_steering;
    rear_steering = state.rear_steering;
    for(int ii=0; ii<state.substate_vec.size(); ii++) {
      InsertSubstate(state.substate_vec[ii]);
    }
  }

  spState& operator=(const spState& rhs) {
    // check for self-assignment
    if(&rhs == this)
        return *this;
    // reuse storage when possible
    pose = rhs.pose;
    linvel = rhs.linvel;
    rotvel = rhs.rotvel;
    wheel_speeds = rhs.wheel_speeds;
    front_steering = rhs.front_steering;
    rear_steering = rhs.rear_steering;
    for(int ii=0; ii<rhs.substate_vec.size(); ii++) {
      InsertSubstate(rhs.substate_vec[ii]);
    }
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const spState& obj)
  {
    os << "**************** State **************"
       << "\nTranslation:\n" << obj.pose.translation().transpose() << std::endl
       << "\nRotation:\n" << obj.pose.rotation() << std::endl
       << "\nLinear Velocity:\n" << obj.linvel.transpose() << std::endl
//       << "\nRotational Velocity:\n" << obj.rotvel.axis()[0] << ", " << obj.rotvel.axis()[1] << ", " << obj.rotvel.axis()[2] << ", " << obj.rotvel.angle() << std::endl
       << "*************************************";
    return os;
  }

  // TODO : check for efficiency
  const spState operator-(const spState &rhs) const {
    spState result;
    result.pose.translate(pose.translation()-rhs.pose.translation());
    result.pose.rotate(pose.rotation()*rhs.pose.rotation().inverse());
    result.linvel = linvel-rhs.linvel;
//    Eigen::Quaterniond rhs_rotvel(rhs.rotvel);
//    Eigen::Quaterniond this_rotvel(rotvel);
//    Eigen::Quaterniond diff_rotvel(this_rotvel*rhs_rotvel.inverse());
//    result.rotvel = Eigen::AngleAxisd(diff_rotvel);
    result.rotvel = rotvel-rhs.rotvel;
//    result.rotvel = rotvel*rhs.rotvel.inverse();
//    result.rotvel.angle() = rotvel.angle()-rhs.rotvel.angle();
//    result.rotvel.axis() = rotvel.axis()-rhs.rotvel.axis();
//    result.front_steering = front_steering - rhs.front_steering;
//    result.rear_steering = rear_steering - rhs.rear_steering;

    return result;
  }

  const spStateVec vector() const{
    spStateVec vec;
    vec[0] = pose.translation()[0];
    vec[1] = pose.translation()[1];
    vec[2] = pose.translation()[2];

    Eigen::AngleAxisd angleaxis(pose.rotation());
    Eigen::Vector3d rotvec(angleaxis.angle()*angleaxis.axis());
    vec[3] = rotvec[0];
    vec[4] = rotvec[1];
    vec[5] = rotvec[2];

    vec[6] = linvel[0];
    vec[7] = linvel[1];
    vec[8] = linvel[2];

//    Eigen::Vector3d rotvelvec(rotvel.angle()*rotvel.axis());
    Eigen::Vector3d rotvelvec(rotvel);
    vec[9] = rotvelvec[0];
    vec[10] = rotvelvec[1];
    vec[11] = rotvelvec[2];

    return vec;
  }

  int InsertSubstate(const spState& substate){
    substate_vec.push_back(std::make_shared<spState>(substate));
    return substate_vec.size()-1;
  }

  int InsertSubstate(std::shared_ptr<spState> substate){
    substate_vec.push_back(std::make_shared<spState>(*substate));
    return substate_vec.size()-1;
  }

  int InsertSubstate(){
    substate_vec.push_back(std::make_shared<spState>());
    return substate_vec.size()-1;
  }

  spPose pose;
  spLinVel linvel;
  spRotVel rotvel;
  spWheelSpeedVec wheel_speeds;
  Eigen::Quaterniond front_steering;
  Eigen::Quaterniond rear_steering;
  std::vector<std::shared_ptr<spState>> substate_vec;
private:
  spStateVec state_vec_;
};

typedef std::chrono::high_resolution_clock::time_point spTimestamp;
typedef Eigen::Matrix4d spMat4x4;

enum class spPhysolver{MLCP_DANTZIG,SEQUENTIAL_IMPULSE,MLCP_PROJECTEDGAUSSSEIDEL};
enum class spGuiType{GUI_NONE,GUI_PANGOSCENEGRAPH};
enum class spPhyEngineType{PHY_NONE,PHY_BULLET};
enum class spObjectType{BOX,VEHICLE_AWSD,WHEEL,WAYPOINT,LINESTRIP};

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
    vehicle_type = spObjectType::VEHICLE_AWSD;
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

  spCurve(int curve_order, int curve_dof): curve_dof_(curve_dof), curve_order_(curve_order) {
    perturbation_value_ = 0;
    perturbation_dim_ = 0;
    // Initialze control points at zero
    ctrl_pts_ = Eigen::ArrayXXd::Zero(curve_dof,curve_order+1);
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

  const Eigen::MatrixXd& GetBezierControlPoints() {
    return ctrl_pts_;
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

  void Get2ndDrivativeCurveArea(Eigen::VectorXd& der_curve_pt) {
    if(curve_order_ == 2) {
      // calculate second derivative of given points
      der_curve_pt = 2*ctrl_pts_.col(2)-4*ctrl_pts_.col(1)+2*ctrl_pts_.col(0);
    } else {
      SPERROREXIT("Not Implemented.");
    }
  }

  void GetPoint(Eigen::VectorXd& point, double t) const {
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
  void GetPoints3d(spPoints3d& pts_vec/*, int num_mid_pts*/) const {
//    int num_pts = num_mid_pts - 1;
    if(pts_vec.size()<1) {
      SPERROREXIT("array size must be greater than zero !");
    }
    int num_pts = pts_vec.size() - 1;
    for (int t = 0; t <= num_pts; t++) {
      spPointXd point(curve_dof_);
      GetPoint(point, t * (1.0 / num_pts));
//      pts_vec.push_back(point.head(3));
      pts_vec[t] = point.head(3);
    }
  }

  // given number of circles find that many curvatures along curve and return max
  double GetMaxCurvature(unsigned int num_circles) {
    SPERROREXIT("not implemented .");
//    spPointsXd points;
//    GetPointsXd(points,2*num_circles+1);

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
