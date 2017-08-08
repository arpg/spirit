#ifndef MPCCOSTFUNC_H__
#define MPCCOSTFUNC_H__
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
#include <thread>
#include <iomanip>
class MPCCostFunc : public ceres::DynamicCostFunction {
 public:
  MPCCostFunc(const spVehicleConstructionInfo& info,
              const spState& current_state,
              const spStateSeries& ref_states,
              const Eigen::VectorXd& state_weight,
              const Eigen::VectorXd& traj_point_weight )
      : vehicle_info_(info), ref_states_(ref_states), current_state_(current_state) {
    // add a 12 vector residual for each state error [x,y,z,p,q,r,x_dot,y_dot,z_dot,p_dot,q_dot,r_dot]
    num_residual_blocks_ = ref_states_.size();

    set_num_residuals(num_residual_blocks_*12);
    // fix a0 and s0 parameters to current value and add parameter block for a1,s1,a2,s2
    AddParameterBlock(6);

    Eigen::VectorXd weightvec(num_residual_blocks_*12);
    for(int ii=0; ii<num_residual_blocks_; ii++) {
      weightvec.block<12,1>(ii*12,0) << traj_point_weight[ii]*state_weight;
    }
    residual_weight_ = weightvec.asDiagonal();
  }

  ~MPCCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    std::vector<std::shared_ptr<spStateSeries>> sim_traj;
    spCtrlPts2ord_2dof cntrl_vars;
    for (int ii = 0; ii < 6; ii++) {
      cntrl_vars.data()[ii] = parameters[0][ii];
    }
    double simulation_length = DISCRETIZATION_STEP_SIZE*ref_states_.size();
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> jac(jacobians[0],num_residual_blocks_*12,6);
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*12,1);
      std::vector<std::shared_ptr<CarSimFunctor>> sims;
      for (int ii = 0; ii < parameter_block_sizes()[0] + 1; ii++) {
        sims.push_back(std::make_shared<CarSimFunctor>(vehicle_info_,current_state_));
      }

#ifdef SOLVER_USE_CENTRAL_DIFF
      std::vector<std::shared_ptr<CarSimFunctor>> sims_neg;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sims_neg.push_back(std::make_shared<CarSimFunctor>(vehicle_info_,current_state_));
      }
#endif

#if 1
      std::vector<std::shared_ptr<std::thread>> thread_vec;
//int ii = 0;
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sim_traj.push_back(std::make_shared<spStateSeries>());
        thread_vec.push_back(std::make_shared<std::thread>(std::ref(*(sims[ii].get())),ii,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, std::ref(cntrl_vars), FINITE_DIFF_EPSILON, ii, sim_traj[ii]));
      }

      sim_traj.push_back(std::make_shared<spStateSeries>());
      thread_vec.push_back(std::make_shared<std::thread>(std::ref(*sims[parameter_block_sizes()[0]].get()),parameter_block_sizes()[0],(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, std::ref(cntrl_vars), FINITE_DIFF_EPSILON, -1,sim_traj[sim_traj.size()-1]));

      for (int ii = 0; ii <= parameter_block_sizes()[0]; ii++) {
          thread_vec[ii]->join();
      }

#elif 1
      std::shared_ptr<CarSimFunctor> sim = std::make_shared<CarSimFunctor>(vehicle_info_,current_state_);
      std::vector<std::shared_ptr<std::thread>> thp;
      thp.push_back(std::make_shared<std::thread>(std::ref(*sims[0].get()),0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, 0));
      if(thp[0]->joinable()) {
        thp[0]->join();
      }
      std::cout << "done here " << std::endl;
#elif 0
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sim_traj.push_back(std::make_shared<spStateSeries>());
        sims[ii]->operator()(0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, ii, sim_traj[ii]);
      }
      sim_traj.push_back(std::make_shared<spStateSeries>());
      sims[parameter_block_sizes()[0]]->operator()(0,(int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1, sim_traj[sim_traj.size()-1]);
#else

      // create thread pool with #n threads
      ctpl::thread_pool pool(2);
      // push tasks to the pool to be executed in thread pool.
      for (int ii = 0; ii < parameter_block_sizes()[0]; ii++) {
        sim_traj.push_back(std::make_shared<spStateSeries>());
        pool.push(std::ref(*sims[ii].get()), (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, FINITE_DIFF_EPSILON, ii, sim_traj[ii]);

#ifdef SOLVER_USE_CENTRAL_DIFF
        sim_traj.push_back(std::make_shared<spStateSeries>());
        pool.push(std::ref(*sims_neg[ii].get()), (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, -FINITE_DIFF_EPSILON, ii, sim_traj[ii]);
#endif

      }
      // run the undisturbed simulation. to be used in finite-diff later
      sim_traj.push_back(std::make_shared<spStateSeries>());
      pool.push(std::ref(*sims[parameter_block_sizes()[0]].get()), (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1, sim_traj[sim_traj.size()-1]);
      // wait untill all threads are done
      pool.stop(true);

#endif

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
          jac.block<12,1>(jj*12,ii) = -(((*(*sim_traj[ii])[jj+1]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj+1])).vector())/FINITE_DIFF_EPSILON;
        }
      }
#endif

      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<12,1>(jj*12,0) = (*(ref_states_[jj]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj+1])).vector();
      }

      // apply weighting matrix
      jac = residual_weight_ * jac;
      res = residual_weight_ * res;

//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << "res   -> \n" << res << std::endl;

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*12,1);
      std::shared_ptr<spStateSeries> curr_states = std::make_shared<spStateSeries>();
      CarSimFunctor sims(vehicle_info_,current_state_);
      sims(0, (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, cntrl_vars, 0, -1, curr_states);
      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<12,1>(jj*12,0) = (*(ref_states_[jj]) - *((*curr_states)[jj+1])).vector();
      }
      res = residual_weight_ * res;
    }
    return true;
  }

 private:
  const spVehicleConstructionInfo vehicle_info_;
  const spState current_state_;
  const spStateSeries& ref_states_;
  Eigen::MatrixXd residual_weight_;
  int num_residual_blocks_;
};

#endif  // MPCCOSTFUNC_H__
