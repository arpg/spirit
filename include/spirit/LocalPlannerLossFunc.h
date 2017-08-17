#ifndef LOCALPLANNERLOSSFUNC_H__
#define LOCALPLANNERLOSSFUNC_H__
#include <iostream>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>

class LocalPlannerLossFunc : public ceres::SizedCostFunction<7, 7> {
 public:
  LocalPlannerLossFunc(double max_steering, double min_steering,
                       double max_throttle, double min_throttle,
                       double max_sim_duration, double min_sim_duration)
      : max_steering_(max_steering),
        min_steering_(min_steering),
        max_throttle_(max_throttle),
        min_throttle_(min_throttle),
        max_sim_duration_(max_sim_duration),
        min_sim_duration_(min_sim_duration) {
    slope_ = 100;
  }

  ~LocalPlannerLossFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> jac(
          jacobians[0]);
      Eigen::Map<Eigen::Matrix<double, 7, 1>> res(residuals);
      // Jacobian calculation
      Eigen::VectorXd jac_vec(7);
      for (int ii = 0; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_steering_) {
          jac_vec[ii] = -slope_;
        } else if (parameters[0][ii] > max_steering_) {
          jac_vec[ii] = slope_;
        } else {
          jac_vec[ii] = 0;
        }
      }
      for (int ii = 1; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_throttle_) {
          jac_vec[ii] = -slope_;
        } else if (parameters[0][ii] > max_throttle_) {
          jac_vec[ii] = slope_;
        } else {
          jac_vec[ii] = 0;
        }
      }
      if (parameters[0][6] < min_sim_duration_) {
        jac_vec[6] = -slope_;
      } else if (parameters[0][6] > max_sim_duration_) {
        jac_vec[6] = slope_;
      } else {
        jac_vec[6] = 0;
      }
      jac = jac_vec.asDiagonal();

      // Residual Calculation
      for (int ii = 0; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_steering_) {
          res[ii] = -slope_ * (parameters[0][ii] - min_steering_);
        } else if (parameters[0][ii] > max_steering_) {
          res[ii] = slope_ * (parameters[0][ii] - max_steering_);
        } else {
          res[ii] = 0;
        }
      }
      for (int ii = 1; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_throttle_) {
          res[ii] = -slope_ * (parameters[0][ii] - min_throttle_);
        } else if (parameters[0][ii] > max_throttle_) {
          res[ii] = slope_ * (parameters[0][ii] - max_throttle_);
        } else {
          res[ii] = 0;
        }
      }
      if (parameters[0][6] < min_sim_duration_) {
        res[6] = -slope_ * (parameters[0][6] - min_sim_duration_);
      } else if (parameters[0][6] > max_sim_duration_) {
        res[6] =  slope_ * (parameters[0][6] - max_sim_duration_);
      } else {
        res[6] = 0;
      }

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double, 7, 1>> res(residuals);
      for (int ii = 0; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_steering_) {
          res[ii] = -slope_ * (parameters[0][ii] - min_steering_);
        } else if (parameters[0][ii] > max_steering_) {
          res[ii] = slope_ * (parameters[0][ii] - max_steering_);
        } else {
          res[ii] = 0;
        }
      }
      for (int ii = 1; ii < 6; ii += 2) {
        if (parameters[0][ii] < min_throttle_) {
          res[ii] = -slope_ * (parameters[0][ii] - min_throttle_);
        } else if (parameters[0][ii] > max_throttle_) {
          res[ii] = slope_ * (parameters[0][ii] - max_throttle_);
        } else {
          res[ii] = 0;
        }
      }
      if (parameters[0][6] < min_sim_duration_) {
        res[6] = -slope_ * (parameters[0][6] - min_sim_duration_);
      } else if (parameters[0][6] > max_sim_duration_) {
        res[6] =  slope_ * (parameters[0][6] - max_sim_duration_);
      } else {
        res[6] = 0;
      }

    }
    return true;
  }

 private:
  double max_steering_;
  double min_steering_;
  double max_throttle_;
  double min_throttle_;
  double max_sim_duration_;
  double min_sim_duration_;
  double slope_;
};

#endif  // LOCALPLANNERLOSSFUNC_H__
