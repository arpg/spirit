#include <spirit/RK4.h>

RK4::RK4(){
  h_ = 0.001;
}

RK4::~RK4(){
}

void RK4::RegisterODE(std::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> ode_func) {
  ode_func_ = ode_func;
}

void RK4::SetStepSize(double step_size) {
  h_ = step_size;
}

Eigen::MatrixXd RK4::Solve(const Eigen::VectorXd& init_cond, Eigen::VectorXd u_t, double horizon) {
  int num_samples = (int)(horizon/h_);
  Eigen::MatrixXd trajectory(init_cond.rows(),num_samples);
  Eigen::VectorXd curr_sol(init_cond);
  for(int ii=0; ii<num_samples; ii++) {
    RK4Iteration(curr_sol,u_t);
    trajectory.col(ii) = curr_sol;
  }
  return trajectory;
}

void RK4::RK4Iteration(Eigen::VectorXd& x_t, Eigen::VectorXd& u_t) {
  Eigen::VectorXd k1,k2,k3,k4;
  k1 = ode_func_(x_t,u_t);
  k2 = ode_func_(x_t+h_*(0.5*k1),u_t);
  k3 = ode_func_(x_t+h_*(0.5*k2),u_t);
  k4 = ode_func_(x_t+h_*k3,u_t);
  x_t = x_t+(h_/6.0)*(k1+2*k2+2*k3+k4,u_t);
}
