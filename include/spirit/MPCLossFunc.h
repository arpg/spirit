#ifndef MPCLOSSFUNC_H__
#define MPCLOSSFUNC_H__
#include <iostream>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <iomanip>

class MPCLossFunc : public ceres::SizedCostFunction<6,6> {
 public:
  MPCLossFunc(double max_steering, double min_steering,
              double max_throttle, double min_throttle)
      : max_steering_(max_steering),
        min_steering_(min_steering),
        max_throttle_(max_throttle),
        min_throttle_(min_throttle){

  }

  ~MPCLossFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,6,1>> res(residuals);

      Eigen::Vector6d min_half;
      Eigen::Vector6d max_half;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        min_half[ii] = std::exp(-parameters[0][ii]-min_steering_);
        max_half[ii] = std::exp(parameters[0][ii]-max_steering_);
      }
      jac = ((-min_half+max_half)*0.1).asDiagonal();

      // Calculate residual
      res = 0.0001*(min_half+max_half);

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,6,1>> res(residuals);
      Eigen::Vector6d min_half;
      Eigen::Vector6d max_half;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        min_half[ii] = std::exp(-parameters[0][ii]-min_steering_);
        max_half[ii] = std::exp(parameters[0][ii]-max_steering_);
      }
      res = 0.0001*(min_half+max_half);
    }
    return true;
  }

 private:
  double max_steering_;
  double min_steering_;
  double max_throttle_;
  double min_throttle_;
};

#endif  // MPCLOSSFUNC_H__
