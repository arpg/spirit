#include <chrono>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <spirit/CalibDerCarODE.h>
#include <spirit/RK4.h>


int main(int argc, char** argv) {

  spTimestamp t0 = spGeneralTools::Tick();

  RK4<CalibDerODE> rk4solver(0.1);
//  rk4solver.RegisterODE(&TestODE);

  Eigen::VectorXd init(11);
  for(int ii=0; ii<11; ii++){
    init[ii] = 0;
  }
  init[0] = 1;

  Eigen::VectorXd pp(14);
  for(int ii=0; ii<14; ii++){
    pp[ii] = 1;
  }
  pp[12] = 0.3;

  rk4solver.SetParameterVec(pp);

  Eigen::VectorXd u(2);
  u[0] = 0.1;
  u[1] = 1;

  std::cout << "here"  << std::endl;
  Eigen::MatrixXd traj;
  Eigen::MatrixXd full_jac(11*(int)(10/0.1),14);
//  for(int ii=0;ii<10;ii++){
    rk4solver.Solve(init,u,10,traj,full_jac);
//  }
  std::cout << "time " << spGeneralTools::Tock_us(t0) << std::endl;
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  std::cout << traj.row(0).transpose() << std::endl;
//  std::cout << "jac is \n" << rk4solver.full_jac_ << std::endl;

  std::cout << "Done ... !" << std::endl;
  return 0;
}
