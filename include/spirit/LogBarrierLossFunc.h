#ifndef LOGBARRIERLOSSFUNC_H__
#define LOGBARRIERLOSSFUNC_H__
#include <iostream>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>

// this loss function penalizes the problem with -epsilon*log(parameter-(limit))
template<int tsize>
class LogBaarrierLossFunc : public ceres::SizedCostFunction<tsize,tsize> {
 public:
  LogBaarrierLossFunc(const Eigen::VectorXd& min_limits, const Eigen::VectorXd& max_limits, double epsilon)
      :  min_limits_(min_limits), max_limits_(max_limits), epsilon_(epsilon) {
    if((min_limits.size() != tsize) || (max_limits.size() != tsize)) {
      SPERROREXIT("Limit vector size doesn't match parameter block size !");
    }
  }

  ~LogBaarrierLossFunc() {}
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    double high_cost = 1000000;
    double high_derivative = 1000;

    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,tsize,tsize,Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::VectorXd jac_vec(tsize);
      for (int ii = 0; ii < tsize; ii++) {
//        if(parameters[0][ii] <= min_limits_[ii]){
//          jac_vec[ii] = -high_derivative;
//        } else if(parameters[0][ii] >= max_limits_[ii]){
//          jac_vec[ii] = high_derivative;
//        } else {
          jac_vec[ii] = -2*epsilon_/(parameters[0][ii]-min_limits_[ii]);
//        }
      }
      jac = jac_vec.asDiagonal();
//      std::cout << "jac vec ->" << jac_vec.transpose() << std::endl;


      for (int ii = 0; ii < tsize; ii++) {
//        if((parameters[0][ii] <= min_limits_[ii]) || (parameters[0][ii] >= max_limits_[ii])){
//          residuals[ii] = high_cost;
//        } else {
          residuals[ii] = -epsilon_*(std::log((parameters[0][ii]-min_limits_[ii])*(max_limits_[ii]-parameters[0][ii])));
//        }
      }
    }
    else if (residuals != NULL) {
      for (int ii = 0; ii < tsize; ii++) {
//        if((parameters[0][ii] <= min_limits_[ii]) || (parameters[0][ii] >= max_limits_[ii])){
//          residuals[ii] = high_cost;
//        } else {
          residuals[ii] = -epsilon_*(std::log((parameters[0][ii]-min_limits_[ii])*(max_limits_[ii]-parameters[0][ii])));
//        }
      }
    }
    return true;
  }

 private:
  Eigen::VectorXd min_limits_;
  Eigen::VectorXd max_limits_;
  double epsilon_;
};

#endif  // LOGBARRIERLOSSFUNC_H__
