#ifndef CANDIDATEWINDOW_H__
#define CANDIDATEWINDOW_H__

#include <spirit/Types/spTypes.h>
#include <ceres/ceres.h>
#include <spirit/Calibration/CalibCostFunc.h>

class CandidateWindow {
 public:
  CandidateWindow(unsigned int window_size) : window_size_(window_size) {
    max_cost_ = 200;
  }

  CandidateWindow(const CandidateWindow& obj) {
    state_series_ = obj.state_series_;
    window_size_ = obj.window_size_;
    entropy_ = obj.entropy_;
    covariance_ = obj.covariance_;
    cov_is_singular_ = obj.cov_is_singular_;
  }

  ~CandidateWindow(){}

  int PushBackState(const spState& state) {
    // add the state to candidate window untill its full. rejected entries (lookup CheckPossibleCandidate memebr function in CandidateWindow class) will return -1
    if(!CheckPossibleCandidate(state)) {
      return -1;
    }
    if(state_series_.size() == window_size_) {
      state_series_.erase(state_series_.begin());
    }
    state_series_.push_back(std::make_shared<spState>(state));
    return state_series_.size();
  }

  void CalculateEntropy(){
    entropy_ = 0.5*std::log((2*SP_PI*SP_EULER*covariance_).determinant());
  }

  bool OptimizeParametersInWindow(const spVehicleConstructionInfo& current_params, Gui* gui){
    Eigen::Vector2d parameter_vec_;
    parameter_vec_[0] = (current_params.wheels_anchor[0]-current_params.wheels_anchor[1])[1];
    parameter_vec_[1] = current_params.wheel_friction;

    ceres::Problem problem;
    Eigen::VectorXd residual_weight(17);
//    residual_weight.setOnes(11);
//    residual_weight << 1,1,1,0.01,0.01,0.01,1E-9,1E-9,1E-9,1E-9,1E-9;
//    residual_weight << 1,1,1,1,1,1,1E-9,1E-9,1E-9,1E-9,1E-9;
//    residual_weight << 0,0,0,0,0,0,1,1,1,1,0;
//    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    Eigen::MatrixXd jacobian;
    ceres::CostFunction* cost_function = new CalibCostFunc(current_params,state_series_,residual_weight,jacobian,gui);

    Eigen::VectorXd min_limits(2);
    Eigen::VectorXd max_limits(2);
    min_limits[0] = 0.2;
    max_limits[0] = 0.4;
    min_limits[1] = 0.1;
    max_limits[1] = 1;

    ceres::CostFunction* loss_function = new ParamLimitLossFunc<2>(min_limits,max_limits,100);

//    parameter_vec_[0] = 0.30;
//    parameter_vec_[1] = 0.6;
    problem.AddResidualBlock(cost_function, NULL, parameter_vec_.data());
    problem.AddResidualBlock(loss_function,NULL,parameter_vec_.data());

//    std::vector<int> fix_param_vec;
//    fix_param_vec.push_back(0);
//    fix_param_vec.push_back(1);

    // Run the solver!
    ceres::Solver::Options options;
    options.initial_trust_region_radius = 5;
    options.max_trust_region_radius = 5;
//    options.min_trust_region_radius = 1e-3;
    options.parameter_tolerance = 1e-3;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.num_linear_solver_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.FullReport() << std::endl;
//    std::cout << "final cost is " << summary.final_cost << std::endl;

    if(summary.final_cost > max_cost_) {
//      std::cout << "max cost reached !" << std::endl;
      return false;
    }
    Eigen::Matrix2d jtj = jacobian.transpose()*jacobian;
    if(jtj.determinant() == 0) {
      cov_is_singular_ = true;
      return false;
    } else {
      cov_is_singular_ = false;
      covariance_ = (jacobian.transpose()*jacobian).inverse();
    }

    // calculate final Covariance
//    ceres::Covariance::Options cov_options;
//    cov_options.num_threads = 1;
//    ceres::Covariance covariance(cov_options);
//    std::vector<const double*> covariance_blocks;
//    covariance_blocks.push_back(parameters);
//    // check if covarinace is non-singular
//    if(!covariance.Compute(covariance_blocks, &problem)) {
//      SPERROREXIT("covariance is singular ");
//      return false;
//    }
//    covariance.GetCovarianceBlock(parameters, parameters, covariance_.data());
//    std::cout << "cov \n" << covariance_ << std::endl;
//    std::cout << "parameters are \t" << parameter_vec_[0] << "\t,\t" << parameter_vec_[1] << std::endl;

//    CalibCarSimFunctor sim(current_params,parameter_vec_[0],parameter_vec_[1],gui);
//    sim(state_series_);
    return true;
  }
  double GetEntropy() const {
    if(cov_is_singular_) {
      return 1000;
    } else {
      return entropy_;
    }
  }

  // check if this state should be added to candidate window
  bool CheckPossibleCandidate(const spState& state) {
    if(state_series_.size()) {
//      if(spGeneralTools::TickTock_ms(state_series_[state_series_.size()-1]->time_stamp,state.time_stamp)<10){
//        return false;
//      }
//      if((state_series_.back()->pose.translation()-state.pose.translation()).norm() < 0.2) {
//        return false;
//      }
    }
    return true;
  }

 public:
  spStateSeries state_series_;

 private:

  unsigned int window_size_;
  double entropy_;
  Eigen::Matrix2d covariance_;
  bool cov_is_singular_;
  double max_cost_;
};

#endif //CANDIDATEWINDOW_H__
