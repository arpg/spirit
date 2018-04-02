#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "CarODE.h"



int main(int argc, char** argv) {

  RK4 rk4solver;
  rk4solver.RegisterODE(&CarODE1);

  Eigen::ArrayXd init(7);
  init[0] = 0.0;
  init[1] = 0.0;
  init[2] = 0.0;
  init[3] = 0.0;
  init[4] = 0.0;
  init[5] = 0.0;
  init[6] = 0.0;

//  Eigen::VectorXd y_dot = TestODE(init);
//  std::cout << "dot " << y_dot << std::endl;
//return 0;

  rk4solver.SetStepSize(0.01);
  Eigen::ArrayXXd traj = rk4solver.Solve(init,20);
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  for(int ii=0; ii<traj.cols(); ii++) {
    std::cout << "res -> " << traj.col(ii).transpose() << std::endl;
  }
//  std::cout << traj.row(0).transpose() << std::endl;
//  std::cout << traj.row(0)[99999] << std::endl;
//  std::cout << traj.row(1)[99999] << std::endl;
//  std::cout << traj.row(2)[99999] << std::endl;

  std::cout << "Done ... !" << std::endl;



  return 0;
}
