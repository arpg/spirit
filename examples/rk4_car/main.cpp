#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include "CarODE.h"

int main(int argc, char** argv) {

  RK4 rk4solver(0.001);
  rk4solver.RegisterODE(&CarODE);

  Eigen::VectorXd init(10);
  init[0] = 0.01;
  init[1] = 0.0;
  init[2] = 0.0;
  init[3] = 0.0;
  init[4] = 0.0;
  init[5] = 0.0;
  init[6] = 0.0;
  init[7] = 0.0;
  init[8] = 0.0;
  init[9] = 0.0;

  Eigen::VectorXd u(2);
  u[0] = SP_PI/10;
  u[1] = 2;

//  Eigen::VectorXd y_dot = TestODE(init);
//  std::cout << "dot " << y_dot << std::endl;
//return 0;

  spTimestamp t0 = spGeneralTools::Tick();
  Eigen::ArrayXXd traj = rk4solver.Solve(init,u,500);
  double calc_time = spGeneralTools::Tock_us(t0);
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  for(int ii=0; ii<traj.cols(); ii++)
  {
    std::cout << traj.col(ii).transpose() << std::endl;
//    std::cout << traj.col(ii)[7] << "," << traj.col(ii)[8] << "," << traj.col(ii)[9] << std::endl;
//    std::cout << traj.col(ii)[0] << "," << traj.col(ii)[1] << "," << traj.col(ii)[2] << "," << traj.col(ii)[7] << "," << traj.col(ii)[8] << "," << traj.col(ii)[9] << std::endl;
  }

  std::cout << "Done ... !" << std::endl;
  std::cout << "calc time is : " << calc_time << std::endl;



  return 0;
}
