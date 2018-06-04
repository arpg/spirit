#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::ArrayXd TestODE(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
//  y_dot = std::tan(y_t[0])+1;
  Eigen::MatrixXd c_mat(2,2);
  c_mat(0,0) = 0.0;
  c_mat(0,1) = 1.0;
  c_mat(1,0) = -1.0;
  c_mat(1,1) = -0.1;
  y_dot = c_mat*y_t;
  return y_dot;
}

int main(int argc, char** argv) {

  RK4 rk4solver(0.01);
  rk4solver.RegisterODE(&TestODE);

  Eigen::VectorXd init(2);
  init[0] = 0.0;
  init[1] = 2.0;

//  Eigen::VectorXd y_dot = TestODE(init);
//  std::cout << "dot " << y_dot << std::endl;
//return 0;

  Eigen::ArrayXXd traj = rk4solver.Solve(init,10);
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  std::cout << traj.row(0).transpose() << std::endl;

  std::cout << "Done ... !" << std::endl;
  return 0;
}
