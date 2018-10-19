#ifndef ODEINTERFACE_H__
#define ODEINTERFACE_H__

#include <eigen3/Eigen/Eigen>

class ODEInterface {
public:
  Eigen::MatrixXd jacobian_;

  virtual const unsigned int StateSpaceSize() = 0;
  virtual const unsigned int ParameterSpaceSize() = 0;
  virtual const unsigned int InputSpaceSize() = 0;
  virtual const bool HasJacobian() = 0;
  virtual void Run(Eigen::VectorXd& x_d, const Eigen::VectorXd& x_t, const Eigen::VectorXd& u_t) = 0;
  virtual void SetParameterVec(const Eigen::VectorXd& pp){
    pp_ = pp;
  }

protected:
  Eigen::VectorXd pp_;
};

#endif // ODEINTERFACE_H__
