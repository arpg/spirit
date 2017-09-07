#ifndef CANDIDATEWINDOW_H__
#define CANDIDATEWINDOW_H__

#include <spirit/Types/spTypes.h>
#include <ceres/ceres.h>
#include <spirit/Calibration/CalibCostFunc.h>
#include <mutex>
#include <atomic>
#include <functional>

class CandidateWindow {
 public:
  CandidateWindow(int window_size, double max_cost, std::shared_ptr<spVehicleConstructionInfo> params, Gui* gui = nullptr) : window_size_(window_size), max_cost_(max_cost), current_params_(params), gui_(gui) {
    thread_should_run_ = false;
    data_loaded_flg_ = false;
    if(gui_ == nullptr) {
      thread_handle_ = std::make_unique<std::thread>(&CandidateWindow::OptimizeParametersInWindow,this);
      thread_should_run_ = true;
    }
  }

  CandidateWindow(const CandidateWindow& obj) {
    state_series_ = obj.state_series_;
    window_size_ = obj.window_size_;
    entropy_ = obj.entropy_;
    covariance_ = obj.covariance_;
    cov_is_singular_ = obj.cov_is_singular_;
    max_cost_ = obj.max_cost_;
    state_series_opt_ = obj.state_series_opt_;
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
    // notify the optimization thread that there is a new data
    {
      std::lock_guard<std::mutex> lock(data_load_mutex_);
      data_loaded_flg_ = true;
    }
    data_load_cv_.notify_one();

    if(gui_!=nullptr) {
      OptimizeParametersInWindow();
    }
    return state_series_.size();
  }


  double GetEntropy() const {
    if(cov_is_singular_) {
      return 1000;
    } else {
      return entropy_;
    }
  }

  Eigen::Vector2d GetEntropyVec() {
    Eigen::Vector2d vec;
    Eigen::EigenSolver<Eigen::Matrix2d> eigensolver(covariance_);
    std::complex<double> eigenvalue0;
    std::complex<double> eigenvalue1;
    eigenvalue0 = eigensolver.eigenvalues().col(0)[0];
    eigenvalue1 = eigensolver.eigenvalues().col(0)[1];
    vec << eigenvalue0.real(),eigenvalue1.real();
    return vec;
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

  bool OptimizeParametersInWindow(){
    while(thread_should_run_) {
      // wait until new state pushed back
      {
        std::unique_lock<std::mutex> lock(data_load_mutex_);
        data_load_cv_.wait(lock,std::bind(&CandidateWindow::GetDataLoadedFlag,this));
        data_loaded_flg_ = false;
      }
      if(state_series_.size() == window_size_){
        // take a copy of state_series_ for optimization. this is to avoid modification from other threads while optimizing
        // since we are just reading the state_series_ object this should be thred safe
        state_series_opt_.clear();
        state_series_opt_ = state_series_;
        params_mutex_.lock();
        current_params_opt_ = current_params_->MakeCopy();
        params_mutex_.unlock();
        // optimize candidate window with new states
        if(OptimizationThread()) {
          std::shared_ptr<CandidateWindow> ptr = std::make_shared<CandidateWindow>(*this);
          if(prqueue_func_ptr_) {
            prqueue_func_ptr_(ptr);
          }
        }
      }
      if(gui_ != nullptr){
        thread_should_run_ = false;
      }
    }
    if(gui_ != nullptr){
      thread_should_run_ = true;
    }

  }

  void SetParams(std::shared_ptr<spVehicleConstructionInfo> params) {
    params_mutex_.lock();
    current_params_ = params->MakeCopy();
    params_mutex_.unlock();
  }

private:
  void CalculateEntropy(){
    entropy_ = 0.5*std::log((2*SP_PI*SP_EULER*covariance_).determinant());
  }

  bool OptimizationThread() {
    Eigen::VectorXd parameter_vec_(current_params_opt_->GetParameterVector());

    ceres::Problem problem;
    Eigen::VectorXd residual_weight(17);
//    residual_weight.setOnes(11);
//    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
//    residual_weight << 1,1,1,1,1,1,1e-5,1e-5,1e-5,1e-5,1e-5,1e-3,1e-3,1e-3,1e-3,1e-3,1e-3;
//    residual_weight << 1,1,1,1,1,1,1e-1,1e-1,1e-1,1e-1,1e-5,1e-1,1e-1,1e-1,1e-3,1e-3,1e-3;
    residual_weight << 1,1,1,0.8,0.8,0.8,0,0,0,0,0,0,0,0,0,0,0;
    Eigen::MatrixXd jacobian;
    ceres::CostFunction* cost_function = new CalibCostFunc(current_params_opt_,state_series_opt_,residual_weight,jacobian,gui_);

    Eigen::VectorXd min_limits(2);
    Eigen::VectorXd max_limits(2);
    min_limits[0] = 0.2;
    max_limits[0] = 0.4;
    min_limits[1] = 0.1;
    max_limits[1] = 1;
    ceres::CostFunction* loss_function = new ParamLimitLossFunc<2>(min_limits,max_limits,100);

    problem.AddResidualBlock(cost_function, NULL, parameter_vec_.data());
    problem.AddResidualBlock(loss_function,NULL,parameter_vec_.data());

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
//      return false;
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

//    CalibCarSimFunctor sim(current_params,parameter_vec_[0],parameter_vec_[1],gui);
//    sim(state_series_);
    CalculateEntropy();
    std::cout << "fist one is done " << std::endl;
    Eigen::EigenSolver<Eigen::Matrix2d> eigensolver(covariance_);
    std::complex<double> eigenvalue0;
    std::complex<double> eigenvalue1;
    eigenvalue0 = eigensolver.eigenvalues().col(0)[0];
    eigenvalue1 = eigensolver.eigenvalues().col(0)[1];
//    std::cout << "eigen values are\t" << eigenvalue0.real()  << "\t,\t" << eigenvalue1.real() << "\t,\t" << summary.final_cost << std::endl;
//        std::cout << "parameters are \t" << parameter_vec_[0] << "\t,\t" << parameter_vec_[1] << "\t,\t" << GetEntropy() << "\t,\t" << summary.final_cost << std::endl;
//    std::cout << "parameters are \t" << covariance_(0,0) << "\t,\t" << covariance_(1,1) << "\t,\t" << GetEntropy() << "\t,\t" << summary.final_cost << std::endl;
    return true;
  }
  bool GetDataLoadedFlag() {
    return data_loaded_flg_;
  }

public:
  spStateSeries state_series_;
  spStateSeries state_series_opt_;
  std::function<int(std::shared_ptr<CandidateWindow>)> prqueue_func_ptr_;

private:
  unsigned int window_size_;
  double entropy_;
  Eigen::Matrix2d covariance_;
  bool cov_is_singular_;
  double max_cost_;
  std::shared_ptr<std::thread> thread_handle_;
  std::atomic<bool> thread_should_run_;
  std::shared_ptr<spVehicleConstructionInfo> current_params_;
  std::shared_ptr<spVehicleConstructionInfo> current_params_opt_;
  std::mutex params_mutex_;

  // synchronization variables
  std::condition_variable data_load_cv_;
  std::mutex data_load_mutex_;
  bool data_loaded_flg_;
  Gui* gui_;
};

#endif //CANDIDATEWINDOW_H__
