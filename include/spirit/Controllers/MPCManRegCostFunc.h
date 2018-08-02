#ifndef MPCMANREGCOSTFUNC_H__
#define MPCMANREGCOSTFUNC_H__
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <spirit/CarSimFunctorRK4.h>
#include <spirit/BikeSimFunctorRK4.h>
#include <spirit/Controllers/CircleMan.h>
#include <spirit/spSimCommonFunctor.h>
#include <thread>
#include <iomanip>

template<typename simfunctor>
class MPCManRegCostFunc : public ceres::DynamicCostFunction {
 public:
  MPCManRegCostFunc(const spVehicleConstructionInfo& info,
              const spState& current_state,
              const Maneuver& ref_man,
              unsigned int num_residual_points,
              const Eigen::VectorXd& state_weight,
              const Eigen::VectorXd& traj_point_weight )
      : vehicle_info_(info), ref_maneuver_(ref_man), current_state_(current_state) {
    // add a 12 vector residual for each state error [x,y,z,p,q,r,x_dot,y_dot,z_dot,p_dot,q_dot,r_dot]
    num_residual_blocks_ = num_residual_points;

    set_num_residuals(num_residual_blocks_*12);
    // fix a0 and s0 parameters to current value and add parameter block for a1,s1,a2,s2
    AddParameterBlock(6);

    Eigen::VectorXd weightvec(num_residual_blocks_*12);
    for(int ii=0; ii<num_residual_blocks_; ii++) {
      weightvec.block<12,1>(ii*12,0) << traj_point_weight[ii]*state_weight;
    }
    residual_weight_ = weightvec.asDiagonal();

//    for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
//      sims.push_back(std::make_shared<CarSimFunctor>(vehicle_info_,current_state_));
//    }

  }

  ~MPCManRegCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    std::vector<std::shared_ptr<spStateSeries>> sim_traj;
    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < 6; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    double simulation_length = DISCRETIZATION_STEP_SIZE*num_residual_blocks_;
    if ((jacobians != NULL) && (residuals != NULL)) {
//      std::cout << "jac called"  << std::endl;
//      std::cout << "parameters are \n" << cntrl_vars << std::endl;
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> jac(jacobians[0],num_residual_blocks_*12,6);
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*12,1);
      std::vector<std::shared_ptr<simfunctor>> sims;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<simfunctor>(vehicle_info_,current_state_));
      }
//      for (int ii = 0; ii < sims.size() ; ii++) {
//        sims[ii]->SetState(current_state_);
//      }
#ifdef SOLVER_USE_CENTRAL_DIFF
      std::vector<std::shared_ptr<simfunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<simfunctor>(vehicle_info_,current_state_));
      }
#endif
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sim_traj.push_back(std::make_shared<spStateSeries>());
//        sims[ii]->RunInThread(ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, ii, sim_traj[ii]);
        (*sims[ii])(ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, ii, sim_traj[ii]);
      }

      sim_traj.push_back(std::make_shared<spStateSeries>());
//      sims[parameter_block_sizes()[0]]->RunInThread(parameter_block_sizes()[0],(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, -1, sim_traj[sim_traj.size()-1]);
      (*sims[parameter_block_sizes()[0]])(parameter_block_sizes()[0],(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, -1, sim_traj[sim_traj.size()-1]);
//      for (int ii = 0; ii <= parameter_block_sizes()[0]; ii++) {
//          sims[ii]->WaitForThreadJoin();
//      }

#ifdef SOLVER_USE_CENTRAL_DIFF
      for (int ii = 0; ii<parameter_block_sizes()[0]; ii++) {
        for(int jj = 0; jj<num_residual_blocks_; jj++) {
          // we have -j since we are calculating (z-h(x)) and then derivative of h(x) would be -J(x)
          jac.block<12,1>(jj*12,ii) = -(((*(*sim_traj[ii])[jj]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj])).vector())/FINITE_DIFF_EPSILON;
        }
      }
#else
      // find Forward difference of residual with respect to parameters
      for (int ii = 0; ii<parameter_block_sizes()[0]; ii++) {
        for(int jj = 0; jj<num_residual_blocks_; jj++) {
          // we have -j since we are calculating (z-h(x)) and then derivative of h(x) would be -J(x)
//          jac.block<12,1>(jj*12,ii) = -(((*(*sim_traj[ii])[jj+1]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj+1])).vector())/FINITE_DIFF_EPSILON;
          jac.block<12,1>(jj*12,ii) = ((ref_maneuver_.GetStateError(*(*sim_traj[ii])[jj+1]) - ref_maneuver_.GetStateError(*(*sim_traj[parameter_block_sizes()[0]])[jj+1])))/FINITE_DIFF_EPSILON;
        }
      }
#endif

      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<12,1>(jj*12,0) = ref_maneuver_.GetStateError(*(*sim_traj[parameter_block_sizes()[0]])[jj+1]);
//        res.block<12,1>(jj*12,0) = (*(ref_states_[jj]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj+1])).vector();
      }

      // apply weighting matrix
      jac = residual_weight_ * jac;
      res = residual_weight_ * res;
//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << "res   -> \n" << res << std::endl;
//      std::cout << "cost is " << res.norm() << std::endl;
//      while(1);
    } else if (residuals != NULL) {
//      std::cout << "res called " << std::endl;
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*12,1);
      std::shared_ptr<spStateSeries> curr_states = std::make_shared<spStateSeries>();
      simfunctor sims(vehicle_info_,current_state_);
      sims(0, (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1, curr_states);
      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<12,1>(jj*12,0) = ref_maneuver_.GetStateError(*((*curr_states)[jj+1]));
//        res.block<12,1>(jj*12,0) = (*(ref_states_[jj]) - *((*curr_states)[jj+1])).vector();
      }
      res = residual_weight_ * res;
//      std::cout << "res   -> \n" << res << std::endl;
//      std::cout << "cost is " << res.norm() << std::endl;
    }
    return true;
  }

 private:
  const spVehicleConstructionInfo& vehicle_info_;
  const spState& current_state_;
  Eigen::MatrixXd residual_weight_;
//  std::vector<std::shared_ptr<CarSimFunctorRK4>> sims;
  int num_residual_blocks_;
  const Maneuver& ref_maneuver_;
};

#endif  // MPCMANREGCOSTFUNC_H__
