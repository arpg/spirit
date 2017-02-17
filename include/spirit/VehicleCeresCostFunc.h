#ifndef VEHICLECERESCOSTFUNC_H__
#define VEHICLECERESCOSTFUNC_H__
#include <spirit/Types/ctpl_stl.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <spirit/CarSimFunctor.h>
#include <iomanip>

/*
struct CarCostFunction {
  CarCostFunction(const spVehicleConstructionInfo& info, const spStateVec&
target_state) : vehicle_info_(info), target_state_(target_state){
  }

  bool operator()(const double* const parameters, double* residual) const {
    spCtrlPts2ord_2dof cntrl_vars;
    for(int ii=0; ii<6; ii++) {
      cntrl_vars.data()[ii] = parameters[ii];
    }
//    std::cout << "params are \n" << std::setprecision(12) << std::fixed <<
cntrl_vars << std::endl;
    Eigen::Map<Eigen::Vector6d> res(residual);
    CarSimFunctor sims(vehicle_info_);
    sims(0,10,0.1,cntrl_vars,0,-1);
    res = target_state_-sims.GetStateVec();
//    std::cout << "res is " << res.transpose() << std::endl;
    return true;
  }
private:
  spVehicleConstructionInfo vehicle_info_;
  spStateVec target_state_;
};
*/

class VehicleCeresCostFunc : public ceres::SizedCostFunction<6, 6> {
 public:
  VehicleCeresCostFunc(const spVehicleConstructionInfo& info,
                       const spStateVec& target_state)
      : vehicle_info_(info), target_state_(target_state) {}

  ~VehicleCeresCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // set finite difference step size
    double epsilon = 0.1;
    // setup a weight vector/matrix
    Eigen::VectorXd vec_diag(6);
    vec_diag << 4, 4, 4, 1, 1, 1;
//    vec_diag << 1, 1, 1, 1, 1, 1;
    Eigen::MatrixXd R = vec_diag.asDiagonal();

    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac(
          jacobians[0]);
      Eigen::Map<Eigen::Vector6d> res(residuals);
      //      std::cout << "doing residual and jac" << std::endl;
      ctpl::thread_pool pool(6);
      std::vector<std::shared_ptr<CarSimFunctor>> sims;
      std::vector<std::shared_ptr<CarSimFunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<CarSimFunctor>(vehicle_info_));
      }
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<CarSimFunctor>(vehicle_info_));
      }
      pool.reinit(6);
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        pool.push(std::ref(*sims[ii].get()), 60, 0.1, cntrl_vars, epsilon, ii);
        pool.push(std::ref(*sims_neg[ii].get()), 60, 0.1, cntrl_vars, -epsilon,
                  ii);
      }
      pool.push(std::ref(*sims[parameter_block_sizes()[0]].get()), 10, 0.1,
                cntrl_vars, 0, -1);
      pool.stop(true);
      pool.clear_queue();
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        // find central difference of residual with respect to parameters
        jac.col(ii) = -(sims[ii]->GetStateVec() - sims_neg[ii]->GetStateVec()) /
                      (2 * (epsilon));
      }
      jac = R * jac;
      res = target_state_ - sims[parameter_block_sizes()[0]]->GetStateVec();
      res = R * res;
    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Vector6d> res(residuals);
      CarSimFunctor sims(vehicle_info_);
      sims(0, 10, 0.1, cntrl_vars, 0, -1);
      res = target_state_ - sims.GetStateVec();
      res = R * res;
    }
    return true;
  }

 private:
  spVehicleConstructionInfo vehicle_info_;
  spStateVec target_state_;
};

#endif  // VEHICLECERESCOSTFUNC_H__
