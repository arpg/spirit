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

class VehicleCeresCostFunc : public ceres::SizedCostFunction<12,7> {
 public:
  VehicleCeresCostFunc(const spVehicleConstructionInfo& info,
                       const spStateVec& target_state)
      : vehicle_info_(info), target_state_(target_state) {}

  ~VehicleCeresCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // set finite difference step size
    double epsilon = 0.05;
    // setup a weight vector/matrix
    Eigen::VectorXd vec_diag(12);
//    vec_diag << 4, 4, 4, 4, 4, 4,1, 1, 1, 1, 1, 1;
//    vec_diag << 1, 1, 1, 1, 1, 1,1, 1, 1, 1, 1, 1;
    vec_diag << 4, 4, 4, 1, 1, 1,1, 1, 1, 0, 0, 0;
//    vec_diag << 4, 4, 4, 4, 4, 4,1, 1, 1, 0, 0, 0;
    Eigen::MatrixXd R = vec_diag.asDiagonal();

    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < 6; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    double simulation_length = parameters[0][6];
//    std::cout << "params -> " << simulation_length << std::endl;
//    std::cout << cntrl_vars << std::endl;
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double, 12, 7, Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,12,1>> res(residuals);
      //      std::cout << "doing residual and jac" << std::endl;
      std::vector<std::shared_ptr<CarSimFunctor>> sims;
      std::vector<std::shared_ptr<CarSimFunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<CarSimFunctor>(vehicle_info_));
      }
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<CarSimFunctor>(vehicle_info_));
      }
      ctpl::thread_pool pool(8);
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        if(ii == parameter_block_sizes()[0]-1) {
          pool.push(std::ref(*sims[ii].get()), (int)(simulation_length/0.1)+1, 0.1, cntrl_vars, 0, -1);
          pool.push(std::ref(*sims_neg[ii].get()), (int)(simulation_length/0.1)-1, 0.1, cntrl_vars, 0,-1);
        } else {
          pool.push(std::ref(*sims[ii].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, epsilon, ii);
          pool.push(std::ref(*sims_neg[ii].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, -epsilon,ii);
        }
      }
      pool.push(std::ref(*sims[parameter_block_sizes()[0]].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, 0, -1);
      pool.stop(true);
//      pool.clear_queue();
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        // find central difference of residual with respect to parameters
        jac.col(ii) = -(sims[ii]->GetStateVec() - sims_neg[ii]->GetStateVec()) /
                      (2 * (epsilon));
      }
      jac = R * jac;
      res = (target_state_ - sims[parameter_block_sizes()[0]]->GetStateVec());
      res = R * res;
//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << "res   -> " << res.transpose() << std::endl;
    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,12,1>> res(residuals);
      CarSimFunctor sims(vehicle_info_);
      sims(0, (int)(simulation_length/0.1), 0.1, cntrl_vars, 0, -1);

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
