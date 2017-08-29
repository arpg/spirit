#ifndef CANDIDATEWINDOW_H__
#define CANDIDATEWINDOW_H__

#include <spirit/Types/spTypes.h>
#include <ceres/ceres.h>
#include <spirit/Calibration/CalibCostFunc.h>

class CandidateWindow {
 public:
  CandidateWindow(unsigned int window_size) : window_size_(window_size) {
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
    entroy_ = 0.5*std::log((2*SP_PI*SP_EULER*covariance_).determinant());
  }

  void OptimizeParametersInWindow(const spVehicleConstructionInfo& init_params, Gui* gui){
    ceres::Problem problem;
    Eigen::VectorXd residual_weight(17);
//    residual_weight.setOnes(11);
//    residual_weight << 1,1,1,0.01,0.01,0.01,1E-9,1E-9,1E-9,1E-9,1E-9;
//    residual_weight << 1,1,1,1,1,1,1E-9,1E-9,1E-9,1E-9,1E-9;
//    residual_weight << 0,0,0,0,0,0,1,1,1,1,0;
//    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    ceres::CostFunction* cost_function = new CalibCostFunc(init_params,state_series_,residual_weight,gui);

//    Eigen::VectorXd min_limits(6);
//    Eigen::VectorXd max_limits(6);
//    for(int ii=0; ii<6; ii+=2) {
//      min_limits[ii] = -SP_PI_QUART;
//      max_limits[ii] = SP_PI_QUART;
//    }
//    for(int ii=1; ii<6; ii+=2) {
//      min_limits[ii] = -200;
//      max_limits[ii] = 200;
//    }
//    ceres::CostFunction* loss_function = new ParamLimitLossFunc<6>(min_limits,max_limits,100);

    double parameters[2];
    parameters[0] = 0.20;
    parameters[1] = 0.5;
    problem.AddResidualBlock(cost_function, NULL, parameters);
//    problem.AddResidualBlock(loss_function,NULL,parameters);

//    std::vector<int> fix_param_vec;
//    fix_param_vec.push_back(0);
//    fix_param_vec.push_back(1);

    // Run the solver!
    ceres::Solver::Options options;
    options.initial_trust_region_radius = 0.8;
    options.max_trust_region_radius = 10;
//    options.min_trust_region_radius = 1e-3;
  //  options.parameter_tolerance = 1e-3;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    // calculate final Covariance
//    ceres::Covariance::Options cov_options;
//    cov_options.num_threads = 1;
//    ceres::Covariance covariance(cov_options);
//    std::vector<const double*> covariance_blocks;
//    covariance_blocks.push_back(parameters);
//    CHECK(covariance.Compute(covariance_blocks, &problem));
//    double covariance_xx[6 * 6];
//    covariance.GetCovarianceBlock(parameters, parameters, covariance_xx);
//    std::cout << "covariance is "  << std::endl;
//    for(int ii=0;ii<6;ii++) {
//      for(int jj=0;jj<6;jj++) {
//        std::cout << "\t" << covariance_xx[ii*6+jj] ;
//      }
//      std::cout  << std::endl;
//    }
//    std::cout << std::endl;

    std::cout << "parameters are \t" << parameters[0] << "\t,\t" << parameters[1] << std::endl;
    SPERROREXIT("done here");
  }
  double GetEntropy(){
    return entroy_;
  }
  // check if this state could be added to candidate window
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


 private:

  unsigned int window_size_;
  double entroy_;
  Eigen::Matrix3d covariance_;
  spStateSeries state_series_;
};

#endif //CANDIDATEWINDOW_H__
