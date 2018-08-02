#ifndef VEHICLECERESCOSTFUNC_H__
#define VEHICLECERESCOSTFUNC_H__
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <spirit/CarSimFunctor.h>
#include <spirit/CarSimFunctorRK4.h>
#include <spirit/BikeSimFunctorRK4.h>
#include <iomanip>

template<typename simfunctor>
class VehicleCeresCostFunc : public ceres::SizedCostFunction<13,7> {
 public:
  VehicleCeresCostFunc(const spVehicleConstructionInfo& info,
                       const spState& current_state,
                       const spState& target_state,
                       const Eigen::VectorXd& residual_weight
                       )
      : vehicle_info_(info), target_state_(target_state), current_state_(current_state) {
    if(residual_weight.size() != 13) {
      SPERROREXIT("residual_weight vector size mismatch.");
    }
    residual_weight_ = residual_weight.asDiagonal();
  }

  ~VehicleCeresCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // setup a weight vector/matrix
    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < 6; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    double simulation_length = parameters[0][6];
//    std::cout << "sim time -> " << simulation_length << std::endl;
//    std::cout << cntrl_vars << std::endl;
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double, 13, 7, Eigen::RowMajor>> jac(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double,13,1>> res(residuals);
      //      std::cout << "doing residual and jac" << std::endl;
      std::vector<std::shared_ptr<simfunctor>> sims;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<simfunctor>(vehicle_info_,current_state_));
      }
#if 0
#ifdef SOLVER_USE_CENTRAL_DIFF
      std::vector<std::shared_ptr<CarSimFunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<simfunctor>(vehicle_info_,current_state_));
      }
#endif
      ctpl::thread_pool pool(1);
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        if(ii == parameter_block_sizes()[0]-1) {
          pool.push(std::ref(*sims[ii].get()), (int)(simulation_length/0.1)+1, 0.1, cntrl_vars, 0, -1);
#ifdef SOLVER_USE_CENTRAL_DIFF
          pool.push(std::ref(*sims_neg[ii].get()), (int)(simulation_length/0.1)-1, 0.1, cntrl_vars, 0,-1);
#endif
        } else {
          pool.push(std::ref(*sims[ii].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, FINITE_DIFF_EPSILON, ii);
#ifdef SOLVER_USE_CENTRAL_DIFF
          pool.push(std::ref(*sims_neg[ii].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, -FINITE_DIFF_EPSILON,ii);
#endif
        }
      }
      pool.push(std::ref(*sims[parameter_block_sizes()[0]].get()), (int)(simulation_length/0.1), 0.1, cntrl_vars, 0, -1);
      pool.stop(true);
#endif

      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        if(ii == parameter_block_sizes()[0]-1) {
          sims[ii]->RunInThread(ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE)+1, DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1);
        } else {
          sims[ii]->RunInThread(ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, ii);
        }
      }
      sims[parameter_block_sizes()[0]]->RunInThread(parameter_block_sizes()[0],(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1);
      for (int ii = 0; ii <= parameter_block_sizes()[0]; ii++) {
          sims[ii]->WaitForThreadJoin();
      }

#ifdef SOLVER_USE_CENTRAL_DIFF
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        // find central difference of residual with respect to parameters
        if(ii == parameter_block_sizes()[0]-1) {
          // we have -j since we are calculating (z-h(x)) and then derivative of h(x) would be -J(x)
          jac.col(ii) << -((sims[ii]->GetState() - sims_neg[ii]->GetState()).vector()) / (2 * (0.1)),1;
        } else {
          jac.col(ii) << -((sims[ii]->GetState() - sims_neg[ii]->GetState()).vector()) / (2 * (FINITE_DIFF_EPSILON)),0;
        }
      }
#else
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        // find central difference of residual with respect to parameters
        if(ii == parameter_block_sizes()[0]-1) {
          // we have -j since we are calculating (z-h(x)) and then derivative of h(x) would be -J(x)
          jac.col(ii) << -((sims[ii]->GetState() - sims[parameter_block_sizes()[0]]->GetState()).vector()) / (DISCRETIZATION_STEP_SIZE),1;
        } else {
          jac.col(ii) << -((sims[ii]->GetState() - sims[parameter_block_sizes()[0]]->GetState()).vector()) / (FINITE_DIFF_EPSILON),0;
        }
      }
#endif
      //  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");;
      //  std::cout << "jacobian is : \n" << jacobian.format(OctaveFmt) << std::endl;
      //  Eigen::MatrixXd jtj(jacobian.transpose()*jacobian);
      //  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jtj);
      //  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
      //  std::cout << "jtj condition number is " << cond << std::endl;

//      std::cout << " jac ->  \n" << jac << std::endl;
      jac = residual_weight_ * jac;
      res << (target_state_ - sims[parameter_block_sizes()[0]]->GetState()).vector() ,simulation_length;
//      std::cout << " target " << target_state_.linvel.transpose() << std::endl;
//      std::cout << " sim    " << sims[parameter_block_sizes()[0]]->GetState().linvel.transpose() << std::endl;
//      std::cout << "res   -> " << res.transpose() << std::endl;
      res = residual_weight_ * res;
//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << "res   -> " << res.transpose() << std::endl;
    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,13,1>> res(residuals);
      simfunctor sims(vehicle_info_,current_state_);
      sims(0, (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1);

      res << (target_state_ - sims.GetState()).vector(),simulation_length;
      res = residual_weight_ * res;
    }
    return true;
  }

 private:
  spVehicleConstructionInfo vehicle_info_;
  spState target_state_;
  spState current_state_;
  Eigen::Matrix<double, 13, 13> residual_weight_;
};

#endif  // VEHICLECERESCOSTFUNC_H__
