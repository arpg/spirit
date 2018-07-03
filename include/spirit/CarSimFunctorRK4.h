#ifndef CARSIMFUNCTORRK4_H__
#define CARSIMFUNCTORRK4_H__
#include <spirit/Types/spTypes.h>
#include <spirit/CarODE.h>
#include <spirit/RK4.h>

class CarSimFunctorRK4 {
 public:
  CarSimFunctorRK4(
      const spVehicleConstructionInfo& info, const spState& initial_state)
      : initial_state_(initial_state),
        thread_(nullptr) {
  }

  ~CarSimFunctorRK4() {
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
        &CarSimFunctorRK4::operator(), this, thread_id, num_sim_steps, step_size,
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
    Eigen::VectorXd init(10);
    RK4 rk4solver(0.01);
    rk4solver.RegisterODE(&CarODE);
    if(init_state != nullptr) {
      initial_state_ = *init_state;
    }
    // rotate from inertial frame to body frame
    double chi = initial_state_.pose.rotation().eulerAngles(0,1,2)[2]+SP_PI_HALF;

    Eigen::Vector2d linvel_chassis;
    // rotate from inertial frame to chassis frame
//    linvel_chassis[1] = std::cos(chi)*initial_state_.linvel[0] - std::sin(chi)*initial_state_.linvel[1];
//    linvel_chassis[0] = std::sin(chi)*initial_state_.linvel[0] + std::cos(chi)*initial_state_.linvel[1];
    linvel_chassis[0] = std::cos(-chi)*initial_state_.linvel[0] - std::sin(-chi)*initial_state_.linvel[1];
    linvel_chassis[1] = std::sin(-chi)*initial_state_.linvel[0] + std::cos(-chi)*initial_state_.linvel[1];

    // vehicle's frame is in traditional NED frame with x as forward and y as right and z down
    if(linvel_chassis[0] == 0){
      // ode can not handle zero forward velocity
      linvel_chassis[0] = 0.0000000001;
    }
    init[0] = linvel_chassis[0];
    init[1] = linvel_chassis[1];
    init[2] = initial_state_.rotvel[2];
    init[3] = initial_state_.wheel_speeds[0];
    init[4] = initial_state_.wheel_speeds[1];
    init[5] = initial_state_.wheel_speeds[2];
    init[6] = initial_state_.wheel_speeds[3];
    init[7] = initial_state_.pose.translation()[0];
    init[8] = initial_state_.pose.translation()[1];
    init[9] = chi;


    spCurve control_curve(2, 2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
    //    spPointXd curvature_cost(2);
    //    control_curve.Get2ndDrivativeCurveArea(curvature_cost);
    if (pert_index >= 0) {
      control_curve.PerturbControlPoint(
          pert_index, epsilon);
    }
    if (traj_states != nullptr) {
      traj_states->push_back(std::make_shared<spState>(initial_state_));
    }
    Eigen::VectorXd u(2);
    for (int ii = 0; ii < num_sim_steps; ii++) {
      control_curve.GetPoint(sample_control, ii / (double)num_sim_steps);
      u[0] = sample_control[0];
      u[1] = sample_control[1];
      rk4solver.Solve(init,u,step_size);
      spState state;
      state.pose = spPose::Identity();
//      int tmp = init[7]/(2*SP_PI);
//      init[7] = init[7]-(tmp*2*SP_PI);
      state.pose.translate(spTranslation(init[7],init[8],initial_state_.pose.translation()[2]));
      Eigen::AngleAxisd rot1(init[9]-SP_PI_HALF,Eigen::Vector3d::UnitZ());
      state.pose.rotate(rot1);
      // rotate back to inertial frame
      state.linvel = spLinVel(0,0,0);
      state.rotvel = spRotVel(0,0,0);
//      state.linvel[0] = std::cos(-init[9])*init[1] - std::sin(-init[9])*init[0];
//      state.linvel[1] = std::sin(-init[9])*init[1] + std::cos(-init[9])*init[0];
      state.linvel[0] = std::cos(init[9])*init[0] - std::sin(init[9])*init[1];
      state.linvel[1] = std::sin(init[9])*init[0] + std::cos(init[9])*init[1];
      state.rotvel[2] = init[2];
      state.wheel_speeds[0] = init[3];
      state.wheel_speeds[1] = init[4];
      state.wheel_speeds[2] = init[5];
      state.wheel_speeds[3] = init[6];
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

#endif  // CARSIMFUNCTORRK4_H__
