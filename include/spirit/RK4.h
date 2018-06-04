#ifndef RK4_H__
#define RK4_H__

#include <functional>
#include <eigen3/Eigen/Eigen>
#include <iostream>

class RK4 {
public:
  RK4(double step_size);
  ~RK4();
  void RegisterODE(std::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> ode_func);
  void RegisterODE(std::function<Eigen::VectorXd(Eigen::VectorXd)> ode_func);
  Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, double horizon);
  Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon);
private:
  void RK4Iteration(Eigen::VectorXd& x_t,const Eigen::VectorXd& u_t);
  void RK4Iteration(Eigen::VectorXd& x_t);
  std::function<Eigen::VectorXd(Eigen::VectorXd)> ode_auto_func_;
  std::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> ode_func_;
  double int_step_;
};

#endif    //RK4_H__
