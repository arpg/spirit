#ifndef PARAMLIMITLOSSFUNC_H__
#define PARAMLIMITLOSSFUNC_H__
#include <iostream>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>

// this loss function penalizes the problem with folowing specs
// if( param < min_limit ) then apply linear cost with negative slope
// if( param < max_limit ) then apply linear cost with positive slope
// if( min_limit < param < max_limit ) then cost is zero
template<int tsize>
class ParamLimitLossFunc : public ceres::SizedCostFunction<tsize,tsize> {
 public:
  ParamLimitLossFunc(const Eigen::VectorXd& min_limits, const Eigen::VectorXd& max_limits, double cost_slope)
      :  min_limits_(min_limits), max_limits_(max_limits), slope_(cost_slope) {
    if((min_limits.size() != tsize) || (max_limits.size() != tsize)) {
      SPERROREXIT("Limit vector size doesn't match parameter block size !");
    }
  }

  ~ParamLimitLossFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,tsize,tsize,Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,tsize,1>> res(residuals);
      // Jacobian calculation
      Eigen::VectorXd jac_vec(tsize);
      for (int ii = 0; ii < tsize; ii++) {
        if(parameters[0][ii]<min_limits_[ii]) {
          jac_vec[ii] = -slope_;
        } else if(parameters[0][ii]>max_limits_[ii]) {
          jac_vec[ii] = slope_;
        } else {
          jac_vec[ii] = 0;
        }
      }
      jac = jac_vec.asDiagonal();
//      std::cout << "jac vec " << jac_vec.transpose() << std::endl;


      // Residual Calculation
      for (int ii = 0; ii < tsize; ii++) {
        if(parameters[0][ii]<min_limits_[ii]) {
          res[ii] = -slope_*(parameters[0][ii]-min_limits_[ii]);
        } else if(parameters[0][ii]>max_limits_[ii]) {
          res[ii] = slope_*(parameters[0][ii]-max_limits_[ii]);
        } else {
          res[ii] = 0;
        }
      }

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,tsize,1>> res(residuals);
      for (int ii = 0; ii < tsize; ii++) {
        if(parameters[0][ii]<min_limits_[ii]) {
          res[ii] = -slope_*(parameters[0][ii]-min_limits_[ii]);
        } else if(parameters[0][ii]>max_limits_[ii]) {
          res[ii] = slope_*(parameters[0][ii]-max_limits_[ii]);
        } else {
          res[ii] = 0;
        }
      }
    }

    return true;
  }

 private:
  Eigen::VectorXd min_limits_;
  Eigen::VectorXd max_limits_;
  double slope_;
};

#endif  // PARAMLIMITLOSSFUNC_H__
