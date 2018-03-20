#ifndef RK4_H__
#define RK4_H__

#include <functional>
#include <eigen3/Eigen/Eigen>
#include <iostream>

class RK4 {
public:
  RK4();
  ~RK4();
  void RegisterODE(std::function<Eigen::VectorXd(Eigen::VectorXd)> ode_func);
  Eigen::MatrixXd Solve(const Eigen::VectorXd& init_cond, double horizon);
  void SetStepSize(double step_size);
private:
  void RK4Iteration(Eigen::VectorXd& x_t);
  std::function<Eigen::VectorXd(Eigen::VectorXd)> ode_func_;
  double h_;
};

#endif    //RK4_H__
