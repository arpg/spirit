#ifndef MESHBIKESIMFUNCTORRK4_H__
#define MESHBIKESIMFUNCTORRK4_H__
#include <spirit/Types/spTypes.h>
#include <spirit/BikeODE.h>
#include <spirit/RK4.h>
#include <spirit/Gui.h>
#include <spirit/spSimCommonFunctor.h>
#include <spirit/Objects/spMesh.h>
#include <spirit/spMeshFunctions.h>
#include <osgDB/ReadFile>

class MeshBikeSimFunctorRK4 : public spSimCommonFunctor {
 public:
  MeshBikeSimFunctorRK4(
      const spVehicleConstructionInfo& info, const spState& initial_state, Gui* gui = nullptr)
      : initial_state_(initial_state),
        thread_(nullptr) {

      rw_ = 0.02;
      rs_ = 0.5;
      pitch_ = 0;

      filesize = 101; // 101 rows and 101 columns

      // load in mesh and interpolant mesh files
      osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
      Vq_ = meshfunc_.ReadFile("Vq.txt", filesize);
      Xq_ = meshfunc_.ReadFile("Xq.txt", filesize);
      Yq_ = meshfunc_.ReadFile("Yq.txt", filesize);
      Nxq_ = meshfunc_.ReadFile("Nx.txt", filesize);
      Nyq_ = meshfunc_.ReadFile("Ny.txt", filesize);
      Nzq_ = meshfunc_.ReadFile("Nz.txt", filesize);


      spMeshVisitor mv;
      meshnode->accept(mv);

      // boundary
      /*
      Eigen::VectorXd b(5);
      b<< -4.5, 2, -1.1, -0.9, 1; //xmin, xmax, ymin, ymax, zlim
      mv.GetMeshData();
      mv.BoundingBox(b);

      v_ = &(*mv.mstruct.bvtx_ptr);
      n_ = &(*mv.mstruct.bnrml_ptr);
       */


  }

  ~MeshBikeSimFunctorRK4() {
    if (thread_ != nullptr) {
      if (thread_->joinable()) {
        thread_->join();
      }
      thread_.reset();
    }
  }

  void RunInThread(int thread_id, double num_sim_steps, double step_size,
                   const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                   int pert_index,
                   std::shared_ptr<spStateSeries> traj_states = nullptr,
                   std::shared_ptr<spState> init_state = nullptr) {
    thread_ = std::make_unique<std::thread>(
        &MeshBikeSimFunctorRK4::operator(), this, thread_id, num_sim_steps, step_size,
        cntrl_vars, epsilon, pert_index, traj_states,init_state);
  }

  void WaitForThreadJoin() {
    thread_->join();
    thread_.reset();
  }

  void operator()(int thread_id, double num_sim_steps, double step_size,
                  const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                  int pert_index,
                  std::shared_ptr<spStateSeries> traj_states = nullptr,
                  std::shared_ptr<spState> init_state = nullptr ) {
    Eigen::VectorXd init(5);
    RK4 rk4solver(0.01);
    rk4solver.RegisterODE(&BikeODE);
    if(init_state != nullptr) {
      initial_state_ = *init_state;
    }

    // init params
    double phi = initial_state_.pose.rotation().eulerAngles(0,1,2)[2];

    initial_state_.linvel[0] = 0;
    initial_state_.linvel[1] = 0;
    initial_state_.linvel[2] = 0;

    init[0] = initial_state_.pose.translation()[0];
    init[1] = initial_state_.pose.translation()[1];
    init[2] = initial_state_.pose.translation()[2];
    init[3] = phi; // yaw
    init[4] = initial_state_.linvel.norm(); // init velocity


    spCurve control_curve(2, 2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
    if (pert_index >= 0) {
      control_curve.PerturbControlPoint(
          pert_index, epsilon);
    }
    if (traj_states != nullptr) {
      traj_states->push_back(std::make_shared<spState>(initial_state_));
    }


    Eigen::VectorXd u(3);
    for(int ii = 0; ii < num_sim_steps; ii++) {
      // control inputs
      control_curve.GetPoint(sample_control, ii / (double)num_sim_steps);
      u[0] = sample_control[0]; // steering
      u[1] = sample_control[1]; // acceleration
      u[2] = pitch_;
      // integrade model
      rk4solver.Solve(init,u,step_size);

      // check points
      //test = meshfunc.MeshDriver(v_,n_,init,rs_,rw_);
      //test = meshfunc_.MeshDriverPrime(v_,n_,init,rs_,rw_);
      //pitch_ = test[0];
      //init[2] = test[1]; // change z position of COG if needed

      // if on quater pipe
      if(init[0] >= -4.5 && init[0] <= -2.5 && init[1] >= -3 && init[1] <= 2){
          // interpolate position on mesh
          Eigen::Vector2d cord = meshfunc_.MeshInterpolate(init);

          double vq = Vq_(cord[1], cord[0]); // x -> cols and y -> rows
          double xq = Xq_(cord[1], cord[0]);
          double yq = Yq_(cord[1], cord[0]);

          Eigen::Vector3d interp_location;
          interp_location << xq, yq, vq;

          // int. normals
          Eigen::Vector3d normal;
          normal[0] = Nxq_(cord[1], cord[0]);
          normal[1] = Nyq_(cord[1], cord[0]);
          normal[2] = Nzq_(cord[1], cord[0]);

          // check conditions
          test = meshfunc_.MeshDriver(normal, init, interp_location, rw_, vq);

          pitch_ = test[0];
          //std::cout<<"Pitch: "<<pitch_<<std::endl;
          init[2] = test[1]; // change z position of COG if needed
      }

      else
      {
          // assume flat surface
          pitch_ = 0;

          // set height to previous height
          init[2] = test[1];
      }



      // update state
      spState state;
      state.pose = spPose::Identity();
      state.pose.translate(spTranslation(init[0],init[1],init[2]));
      Eigen::AngleAxisd rot1(init[3],Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd rot2(pitch_,Eigen::Vector3d::UnitY());
      //Eigen::Matrix3d m = Eigen::AngleAxisd(init[3],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(pitch_,Eigen::Vector3d::UnitX());;
      state.pose.rotate(rot1);
      state.pose.rotate(rot2);
      state.front_steering = u[0];
      state.linvel = spLinVel(0,0,0);
      //state.rotvel = spRotVel(0,0,0);
      //state.rotvel[2] = 1;
      state.linvel[0] = init[4]*std::cos(init[3]);
      state.linvel[1] = init[4]*std::sin(init[3]);
      state.linvel[2] = init[4]*std::sin(pitch_);
      //std::cout<<"the pitch: "<<test[0]<<std::endl;
      //std::cout<<"the z pos: "<<init[2]<<std::endl;


      if (traj_states != nullptr) {
        traj_states->push_back(std::make_shared<spState>(state));
      }
      curr_state_ = state;
      if(init_state != nullptr) {
        *init_state = state;
      }
    }
  }

  const spState& GetState() {
    return curr_state_;
  }

  void SetMeshData(Eigen::MatrixXd *vtx, Eigen::MatrixXd *nrml){
    v_ = &(*vtx);
    n_ = &(*nrml);
    std::cout<<"Vertices: "<<(*v_).rows()<<" "<<"Normals: "<<(*n_).rows()<<std::endl;
  }



 public:
  spCtrlPts3ord_3dof traj_curve_;
  spState initial_state_;
  std::unique_ptr<std::thread> thread_;
  spState curr_state_;

  spMeshFunctions meshfunc_;


  Eigen::MatrixXd Vq_;
  Eigen::MatrixXd Xq_;
  Eigen::MatrixXd Yq_;

  Eigen::MatrixXd Nxq_;
  Eigen::MatrixXd Nyq_;
  Eigen::MatrixXd Nzq_;

  Eigen::MatrixXd *v_;
  Eigen::MatrixXd *n_;
  Eigen::VectorXd index_;
  Eigen::VectorXd angles_;
  Eigen::VectorXd vnear_; // nearest vertex
  Eigen::VectorXd params_;
  Eigen::Vector3d test;
  double testpitch_;
  double pitch_; // pitch of bike
  double rw_; // wheel
  double rs_; // radius of search for vetices

  int filesize;

};

#endif  // MESHBIKESIMFUNCTORRK4_H__
