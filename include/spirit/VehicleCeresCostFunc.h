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

class VehicleCeresCostFunc : public ceres::SizedCostFunction<13,7> {
 public:
  VehicleCeresCostFunc(const spVehicleConstructionInfo& info,
                       const spState& current_state,
                       const spState& target_state)
      : vehicle_info_(info), target_state_(target_state), current_state_(current_state) {}

  ~VehicleCeresCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // set finite difference step size. tests shows that 0.05 is a good minimum step size
    double epsilon = 0.05;
    // setup a weight vector/matrix
    Eigen::VectorXd vec_diag(13);
//    vec_diag << 2, 2, 2, 2, 2, 2,0.09, 0.09, 0.09, 1, 1, 1,0.5;
    vec_diag << 3, 3, 3, 3, 3, 3,0.07, 0.07, 0.07, 1, 1, 1,0.5;
    Eigen::MatrixXd R = vec_diag.asDiagonal();

    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < 6; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    double simulation_length = parameters[0][6];
    std::cout << "sim time -> " << simulation_length << std::endl;
//    std::cout << cntrl_vars << std::endl;
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double, 13, 7, Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,13,1>> res(residuals);
      //      std::cout << "doing residual and jac" << std::endl;
      std::vector<std::shared_ptr<CarSimFunctor>> sims;
      std::vector<std::shared_ptr<CarSimFunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<CarSimFunctor>(vehicle_info_,current_state_));
      }
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<CarSimFunctor>(vehicle_info_,current_state_));
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
        if(ii == parameter_block_sizes()[0]-1) {
          jac.col(ii) << -((sims[ii]->GetState() - sims_neg[ii]->GetState()).vector()) / (2 * (0.1)),1;
        } else {
          jac.col(ii) << -((sims[ii]->GetState() - sims_neg[ii]->GetState()).vector()) / (2 * (epsilon)),0;
        }
      }

      //  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");;
      //  std::cout << "jacobian is : \n" << jacobian.format(OctaveFmt) << std::endl;
      //  Eigen::MatrixXd jtj(jacobian.transpose()*jacobian);
      //  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jtj);
      //  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
      //  std::cout << "jtj condition number is " << cond << std::endl;

//      std::cout << " jac ->  \n" << jac << std::endl;
      jac = R * jac;
      res << (target_state_ - sims[parameter_block_sizes()[0]]->GetState()).vector() ,simulation_length;
//      std::cout << " target " << target_state_.linvel.transpose() << std::endl;
//      std::cout << " sim    " << sims[parameter_block_sizes()[0]]->GetState().linvel.transpose() << std::endl;
//      std::cout << "res   -> " << res.transpose() << std::endl;
      res = R * res;
//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << "res   -> " << res.transpose() << std::endl;
    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,13,1>> res(residuals);
      CarSimFunctor sims(vehicle_info_,current_state_);
      sims(0, (int)(simulation_length/0.1), 0.1, cntrl_vars, 0, -1);

      res << (target_state_ - sims.GetState()).vector(),simulation_length;
      res = R * res;
    }
    return true;
  }

 private:
  spVehicleConstructionInfo vehicle_info_;
  spState target_state_;
  spState current_state_;
};

#endif  // VEHICLECERESCOSTFUNC_H__
