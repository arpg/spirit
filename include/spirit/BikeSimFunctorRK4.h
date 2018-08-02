#ifndef BIKESIMFUNCTORRK4_H__
#define BIKESIMFUNCTORRK4_H__
#include <spirit/Types/spTypes.h>
#include <spirit/BikeODE.h>
#include <spirit/RK4.h>
#include <spirit/Gui.h>
#include <spirit/spSimCommonFunctor.h>

class BikeSimFunctorRK4 : public spSimCommonFunctor {
 public:
  BikeSimFunctorRK4(
      const spVehicleConstructionInfo& info, const spState& initial_state, Gui* gui = nullptr)
      : initial_state_(initial_state),
        thread_(nullptr) {
  }

  ~BikeSimFunctorRK4() {
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
        &BikeSimFunctorRK4::operator(), this, thread_id, num_sim_steps, step_size,
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
    Eigen::VectorXd init(4);
    RK4 rk4solver(0.01);
    rk4solver.RegisterODE(&BikeODE);
    if(init_state != nullptr) {
      initial_state_ = *init_state;
    }

    double phi = initial_state_.pose.rotation().eulerAngles(0,1,2)[2];

    initial_state_.linvel[0] = 0;
    initial_state_.linvel[1] = 0;
    initial_state_.linvel[2] = 0;

    init[0] = initial_state_.pose.translation()[0];
    init[1] = initial_state_.pose.translation()[1];
    init[2] = phi; // yaw
    init[3] = initial_state_.linvel.norm(); // init velocity


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


    Eigen::VectorXd u(2);
    for (int ii = 0; ii < num_sim_steps; ii++) {
      // control inputs
      control_curve.GetPoint(sample_control, ii / (double)num_sim_steps);
      u[0] = sample_control[0]; // steering
      u[1] = sample_control[1]; // acceleration
      rk4solver.Solve(init,u,step_size);
      spState state;
      state.pose = spPose::Identity();
      state.pose.translate(spTranslation(init[0],init[1],initial_state_.pose.translation()[2]));
      Eigen::AngleAxisd rot1(init[2],Eigen::Vector3d::UnitZ());
      state.pose.rotate(rot1);
      state.front_steering = u[0];
      state.linvel = spLinVel(0,0,0);
      //state.rotvel = spRotVel(0,0,0);
      //state.rotvel[2] = 1;
      state.linvel[0] = init[3]*std::cos(init[2]);
      state.linvel[1] = init[3]*std::sin(init[2]);

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

 public:
  spCtrlPts3ord_3dof traj_curve_;
  spState initial_state_;
  std::unique_ptr<std::thread> thread_;
  spState curr_state_;
};

#endif  // BIKESIMFUNCTORRK4_H__
