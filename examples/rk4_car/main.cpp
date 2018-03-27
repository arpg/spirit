#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::ArrayXd TestODE(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
//////////////////////

//////////////////////
  return y_dot;
}

int main(int argc, char** argv) {

  RK4 rk4solver;
  rk4solver.RegisterODE(&TestODE);

  Eigen::ArrayXd init(2);
  init[0] = 0.0;
  init[1] = 2.0;

//  Eigen::VectorXd y_dot = TestODE(init);
//  std::cout << "dot " << y_dot << std::endl;
//return 0;

  rk4solver.SetStepSize(0.01);
  Eigen::ArrayXXd traj = rk4solver.Solve(init,10);
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  std::cout << traj.row(0).transpose() << std::endl;

  std::cout << "Done ... !" << std::endl;
  return 0;
}
