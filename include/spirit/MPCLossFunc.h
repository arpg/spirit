#ifndef MPCLOSSFUNC_H__
#define MPCLOSSFUNC_H__
#include <iostream>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>

class MPCLossFunc : public ceres::SizedCostFunction<6,6> {
 public:
  MPCLossFunc(double max_steering, double min_steering,
              double max_throttle, double min_throttle)
      : max_steering_(max_steering),
        min_steering_(min_steering),
        max_throttle_(max_throttle),
        min_throttle_(min_throttle){
    slope_ = 100;
  }

  ~MPCLossFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,6,1>> res(residuals);
      // Jacobian calculation
      Eigen::Vector6d jac_vec;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_steering_) {
          jac_vec[ii] = -slope_;
        } else if(parameters[0][ii]>max_steering_) {
          jac_vec[ii] = slope_;
        } else {
          jac_vec[ii] = 0;
        }
      }
      for (int ii = 1; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_throttle_) {
          jac_vec[ii] = -slope_;
        } else if(parameters[0][ii]>max_throttle_) {
          jac_vec[ii] = slope_;
        } else {
          jac_vec[ii] = 0;
        }
      }
      jac = jac_vec.asDiagonal();
      // Residual Calculation
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_steering_) {
          res[ii] = -slope_*(parameters[0][ii]-min_steering_);
        } else if(parameters[0][ii]>max_steering_) {
          res[ii] = slope_*(parameters[0][ii]-max_steering_);
        } else {
          res[ii] = 0;
        }
      }
      for (int ii = 1; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_throttle_) {
          res[ii] = -slope_*(parameters[0][ii]-min_throttle_);
        } else if(parameters[0][ii]>max_throttle_) {
          res[ii] = slope_*(parameters[0][ii]-max_throttle_);
        } else {
          res[ii] = 0;
        }
      }

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,6,1>> res(residuals);
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_steering_) {
          res[ii] = -slope_*(parameters[0][ii]-min_steering_);
        } else if(parameters[0][ii]>max_steering_) {
          res[ii] = slope_*(parameters[0][ii]-max_steering_);
        } else {
          res[ii] = 0;
        }
      }
      for (int ii = 1; ii < parameter_block_sizes()[0]; ii+=2) {
        if(parameters[0][ii]<min_throttle_) {
          res[ii] = -slope_*(parameters[0][ii]-min_throttle_);
        } else if(parameters[0][ii]>max_throttle_) {
          res[ii] = slope_*(parameters[0][ii]-max_throttle_);
        } else {
          res[ii] = 0;
        }
      }
    }
    return true;
  }

 private:
  double max_steering_;
  double min_steering_;
  double max_throttle_;
  double min_throttle_;
  double slope_;
};

#endif  // MPCLOSSFUNC_H__
