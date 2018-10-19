#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

class TestODE : public ODEInterface{
public:
  const unsigned int StateSpaceSize(){
    return 2;
  }

  const unsigned int ParameterSpaceSize(){
    return 0;
  }

  const unsigned int InputSpaceSize(){
    return 0;
  }

  const bool HasJacobian(){
    return false;
  }

  void Run(Eigen::VectorXd& x_d_t, const Eigen::VectorXd& x_t, const Eigen::VectorXd& u_t){
//    Eigen::VectorXd x_dot(x_t);
  //  y_dot = std::tan(y_t[0])+1;
    Eigen::MatrixXd c_mat(2,2);
    c_mat(0,0) = 0.0;
    c_mat(0,1) = 1.0;
    c_mat(1,0) = -1.0;
    c_mat(1,1) = -0.1;
    x_d_t = c_mat*x_t;
//    return x_dot;
  }
};

int main(int argc, char** argv) {

  RK4<TestODE> rk4solver(0.01);
//  rk4solver.RegisterODE(&TestODE);

  Eigen::VectorXd init(2);
  init[0] = 0.0;
  init[1] = 2.0;

//  Eigen::VectorXd y_dot = TestODE(init);
//  std::cout << "dot " << y_dot << std::endl;
//return 0;
  std::cout << "here"  << std::endl;
  Eigen::MatrixXd traj = rk4solver.Solve(init,10);
  std::cout << "traj cols " << traj.cols() << std::endl;
  std::cout << "traj rows " << traj.rows() << std::endl;
  std::cout << traj.row(0).transpose() << std::endl;

  std::cout << "Done ... !" << std::endl;
  return 0;
}
