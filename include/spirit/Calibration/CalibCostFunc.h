#ifndef CALIBCOSTFUNC_H__
#define CALIBCOSTFUNC_H__
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <spirit/spirit.h>
#include <spirit/Calibration/CalibCarSimFunctor.h>
#include <thread>

class CalibCostFunc : public ceres::DynamicCostFunction {
 public:
  CalibCostFunc(const spVehicleConstructionInfo& info,
              const spStateSeries& ref_states,
              const Eigen::VectorXd& state_weight,
                Gui* gui)
      : vehicle_info_(info), ref_states_(ref_states) , gui_(gui){
    // don't include the very first state in residual since error is gonna be zero for that term always
    num_residual_blocks_ = ref_states_.size()-1;

    set_num_residuals(num_residual_blocks_*17);
    AddParameterBlock(2);

    Eigen::VectorXd weightvec(num_residual_blocks_*17);
    for(int ii=0; ii<num_residual_blocks_; ii++) {
      weightvec.block<17,1>(ii*17,0) << state_weight;
    }
    residual_weight_ = weightvec.asDiagonal();

  }

  ~CalibCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians)  const {

    if ((jacobians != NULL) && (residuals != NULL)) {
      std::vector<std::shared_ptr<spStateSeries>> sim_traj;
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> jac(jacobians[0],num_residual_blocks_*17,2);
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*17,1);
      double epsilon = 0.1;
      std::vector<std::shared_ptr<CalibCarSimFunctor>> sims;
      std::cout << std::fixed << std::setprecision(12) << "params are " << parameters[0][0] << "\t\t" << parameters[0][1] << std::endl;

      sims.push_back(std::make_shared<CalibCarSimFunctor>(vehicle_info_,parameters[0][0]+epsilon,parameters[0][1]));
      sims.push_back(std::make_shared<CalibCarSimFunctor>(vehicle_info_,parameters[0][0],parameters[0][1]+epsilon));
      sims.push_back(std::make_shared<CalibCarSimFunctor>(vehicle_info_,parameters[0][0],parameters[0][1],gui_));

      for (int ii = 0; ii < parameter_block_sizes()[0]+1; ii++) {
        sim_traj.push_back(std::make_shared<spStateSeries>());
//        sims[ii]->RunInThread(ref_states_, sim_traj[ii]);
        sims[ii]->operator()(ref_states_, sim_traj[ii]);
      }

      // Enable folowing loop if using threaded simulation
//      for (int ii = 0; ii < parameter_block_sizes()[0]+1; ii++) {
//          sims[ii]->WaitForThreadJoin();
//      }

      // find Forward difference of residual with respect to parameters
      for (int ii = 0; ii<parameter_block_sizes()[0]; ii++) {
        for(int jj = 0; jj<num_residual_blocks_; jj++) {
          // we have -j since we are calculating (z-h(x)) and then derivative of h(x) would be -J(x)
          jac.block<17,1>(jj*17,ii) = -(((*(*sim_traj[ii])[jj]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj])).calibvector())/epsilon;
        }
      }
      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<17,1>(jj*17,0) = (*(ref_states_[jj+1]) - (*(*sim_traj[parameter_block_sizes()[0]])[jj])).calibvector();
      }

      // apply weighting matrix
      jac = residual_weight_ * jac;
      res = residual_weight_ * res;
//      std::cout << " jac ->  \n" << jac << std::endl;
//      std::cout << " res -> \n" << res << std::endl;
//      std::cout << "cost is " << res.norm() << std::endl;
    } else if (residuals != NULL) {
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> res(residuals,num_residual_blocks_*17,1);
      std::shared_ptr<spStateSeries> curr_states = std::make_shared<spStateSeries>();
      CalibCarSimFunctor sims(vehicle_info_,parameters[0][0],parameters[0][1]);
      sims(ref_states_, curr_states);
      // Calculate residual
      for(int jj = 0; jj<num_residual_blocks_; jj++) {
        res.block<17,1>(jj*17,0) = (*(ref_states_[jj+1]) - *((*curr_states)[jj])).calibvector();
      }
      res = residual_weight_ * res;
//      std::cout << "res   -> \n" << res << std::endl;
//      std::cout << "cost is " << res.norm() << std::endl;
    }
    return true;
  }

 private:
  Gui* gui_;
  const spVehicleConstructionInfo& vehicle_info_;
  const spStateSeries& ref_states_;
  Eigen::MatrixXd residual_weight_;
  int num_residual_blocks_;
};

#endif  // CALIBCOSTFUNC_H__
