#ifndef ENTROPYTABLE_H__
#define ENTROPYTABLE_H__

#include <spirit/Calibration/CandidateWindow.h>
#include <condition_variable>
typedef Eigen::Vector2d Vec;

class EntropyTable {
 public:
  EntropyTable(const spVehicleConstructionInfo& car_params, int num_params, int candidates_per_param): current_params_(car_params), num_params_(num_params),candidates_per_param_(candidates_per_param) {
    alpha_ = 0.95;
    thread_should_run_ = true;
    data_loaded_flg_ = false;
    thread_handle_ = std::make_unique<std::thread>(&EntropyTable::OptimizeParametersInQueue,this);
  }
  ~EntropyTable(){}
  int PushBackCandidateWindow(std::shared_ptr<CandidateWindow> candidate) {
    // accept very firt candidate
    if(table_.size() == 0) {
      Vec ones;
      ones << 1,1;
      table_.push_back(std::make_pair(candidate,ones));
      score_count_ = ones;
      FindHighestEntropy();
      return 0;
    }

    // check if this is a better candidate than the ones already in the list
//    std::cout << "entropy is " << max_entropy_.transpose() << std::endl;
    int candidate_score = 0;
    Vec candidate_score_vec = Vec::Zero();
    for(int jj=0; jj<num_params_; jj++) {
      if(candidate->GetEntropyVec()[jj] < alpha_*max_entropy_[jj]) {
        candidate_score++;
        candidate_score_vec[jj] = 1;
      }
    }
    if(candidate_score == 0) {
      return 0;
    }

    // check if this candidate has common segment with any candidate in queue
    int common_index = FindCommonCandidate(candidate);
    if(common_index == -1) {
      // if queue is full then replace it with highest entropy
      for(int jj=0; jj<num_params_; jj++) {
        if(candidate_score_vec[jj]) {
          if(score_count_[jj] >= candidates_per_param_) {
            table_[max_entropy_index_[jj]].second[jj] = 0;
            score_count_[jj]--;
          }
        }
      }
    } else {
      int prev_score = 0;
      for(int jj=0; jj<num_params_; jj++) {
        prev_score += table_[common_index].second[jj];
      }
      candidate_score = 0;
      candidate_score_vec = Vec::Zero();
      for(int jj=0; jj<num_params_; jj++) {
        if(candidate->GetEntropyVec()[jj] < alpha_*table_[common_index].first->GetEntropyVec()[jj]) {
          if(score_count_[jj] < candidates_per_param_) {
            candidate_score++;
            candidate_score_vec[jj] = 1;
          }
        }
      }
//      if(!(candidate->GetEntropy() < alpha_*table_[common_index].first->GetEntropy())) {
      if(candidate_score < prev_score){
        return 0;
      }
      score_count_ -= table_[common_index].second;
      table_.erase(table_.begin()+common_index);
    }
    // push new candiate to queue
    table_.push_back(std::make_pair(candidate,candidate_score_vec));
    score_count_ += candidate_score_vec;
    PruneTable();
    FindHighestEntropy();
//    CoutTable();

    // notify the optimization thread that there is a new data
    {
      std::lock_guard<std::mutex> lock(data_load_mutex_);
      data_loaded_flg_ = true;
      data_load_cv_.notify_one();
    }
//    thread_should_run_ = true;
//    OptimizeParametersInQueue();
    return 0;
  }

  bool GetParameters(spVehicleConstructionInfo& car_params) {
    if(current_param_mutex_.try_lock()) {
      car_params = current_params_;
      current_param_mutex_.unlock();
      return true;
    }
    return false;
  }


private:
  bool GetDataLoadedFlag(){
    return data_loaded_flg_;
  }

  void OptimizeParametersInQueue() {
    while(thread_should_run_) {
      // wait until new candidate window pushed back
      {
        std::unique_lock<std::mutex> lock(data_load_mutex_);
        data_load_cv_.wait(lock,std::bind(&EntropyTable::GetDataLoadedFlag,this));
        data_loaded_flg_ = false;
      }
      // take a copy of queue_ for optimization. this is to avoid queue modification from other threads while optimizing
      // since we are just reading the queue_ object this should be thred safe
      queue_opt_.clear();
      for(int ii=0; ii<table_.size(); ii++) {
        queue_opt_.push_back(table_[ii].first);
      }
      std::cout << "opt" << std::endl;
      // optimize queue with new candidate windows
      OptimizationThread();
//      thread_should_run_ = false;
    }
  }

  double OptimizationThread() {
    Eigen::Vector2d parameter_vec;
    ceres::Problem problem;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd residual_weight(17);
//    residual_weight << 1,1,1,1,1,1,1e-5,1e-5,1e-5,1e-5,1,1,1,1,1,1,1;
//    residual_weight << 1,1,1,1,1,1,1e-5,1e-5,1e-5,1e-5,1e-5,1e-3,1e-3,1e-3,1e-3,1e-3,1e-3;
//    residual_weight << 1,1,1,1,1,1,1e-1,1e-1,1e-1,1e-1,1e-5,1e-1,1e-1,1e-1,1e-3,1e-3,1e-3;
//    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    residual_weight << 1,1,1,0.8,0.8,0.8,0,0,0,0,0,0,0,0,0,0,0;
    current_param_mutex_.lock();
    parameter_vec[0] = (current_params_.wheels_anchor[0]-current_params_.wheels_anchor[1])[1];
    parameter_vec[1] = current_params_.wheel_friction;
    for(int ii=0; ii<queue_opt_.size(); ii++) {
      ceres::CostFunction* cost_function = new CalibCostFunc(current_params_,queue_opt_[ii]->state_series_opt_,residual_weight,jacobian);
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), parameter_vec.data());
    }
    Eigen::VectorXd min_limits(2);
    Eigen::VectorXd max_limits(2);
    min_limits[0] = 0.1;
    max_limits[0] = 0.5;
    min_limits[1] = 0.1;
    max_limits[1] = 1;
    ceres::CostFunction* loss_function = new ParamLimitLossFunc<2>(min_limits,max_limits,100);
    problem.AddResidualBlock(loss_function,NULL,parameter_vec.data());

//    std::vector<int> fix_param_vec;
//    fix_param_vec.push_back(0);
//    fix_param_vec.push_back(1);

    // Run the solver!
    ceres::Solver::Options options;
    options.initial_trust_region_radius = 5;
    options.max_trust_region_radius = 5;
//    options.min_trust_region_radius = 1e-3;
    options.parameter_tolerance = 1e-4;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.num_linear_solver_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.FullReport() << std::endl;

    // calculate final Covariance
    Eigen::Matrix2d covariance_;
    ceres::Covariance::Options cov_options;
    cov_options.num_threads = 1;
    ceres::Covariance covariance(cov_options);
    std::vector<const double*> covariance_blocks;
    covariance_blocks.push_back(parameter_vec.data());
    // check if covarinace is non-singular
    if(!covariance.Compute(covariance_blocks, &problem)) {
      SPERROREXIT("covariance is singular ");
      return false;
    }
    covariance.GetCovarianceBlock(parameter_vec.data(), parameter_vec.data(), covariance_.data());
//    std::cout << "cov \n" << covariance_ << std::endl;
    std::cout << parameter_vec[0] << "," << parameter_vec[1]  << "," << covariance_(0,0) << "," << covariance_(1,1) << ";" << std::endl;

    current_params_.wheels_anchor[0][1] = parameter_vec[0]/2;
    current_params_.wheels_anchor[1][1] = -parameter_vec[0]/2;
    current_params_.wheels_anchor[2][1] = -parameter_vec[0]/2;
    current_params_.wheels_anchor[3][1] = parameter_vec[0]/2;
    current_params_.wheel_friction = parameter_vec[1];
    current_param_mutex_.unlock();
  }

  void CoutTable() {
    std::cout << "***************** Table is ****************" << std::endl;
    for(int ii=0; ii<table_.size(); ii++) {
      std::cout << table_[ii].first->GetEntropyVec().transpose() << "\t|\t";
      std::cout << table_[ii].first->GetEntropy() << "\t|\t";
      std::cout << table_[ii].second.transpose() << std::endl;
    }
    std::cout << "score count is " << score_count_.transpose() << std::endl;
  }

  void FindHighestEntropy() {
    max_entropy_index_ = Vec::Zero();
    max_entropy_ = Vec::Zero();
    for(int ii=0; ii<table_.size(); ii++) {
      Vec entropy_vec = table_[ii].first->GetEntropyVec();
      for(int jj=0; jj<num_params_; jj++) {
        if(table_[ii].second[jj]){
          if(entropy_vec[jj] > max_entropy_[jj]) {
            max_entropy_[jj] = entropy_vec[jj];
            max_entropy_index_[jj] = ii;
          }
        }
      }
    }
  }

  int FindCommonCandidate(std::shared_ptr<CandidateWindow> candidate) {
    for(int ii=0; ii<table_.size(); ii++) {
      // last timestamp of ii'th candidate in queue
      spTimestamp t0 = table_[ii].first->state_series_opt_.back()->time_stamp;
      // first timestamp of given candidate
      spTimestamp t1 = candidate->state_series_opt_.front()->time_stamp;
      if(t0>=t1) {
        return ii;
      }
    }
    return -1;
  }

  void PruneTable() {
    for(int ii=0; ii<table_.size(); ii++) {
      int candidate_score = 0;
      for(int jj=0; jj<num_params_; jj++) {
          candidate_score += table_[ii].second[jj];
      }
      if(candidate_score == 0) {
        table_.erase(table_.begin()+ii);
      }
    }
  }



  spVehicleConstructionInfo current_params_;
  std::mutex current_param_mutex_;
  std::vector<std::pair<std::shared_ptr<CandidateWindow>,Vec>> table_;
  std::vector<std::shared_ptr<CandidateWindow>> queue_opt_;
  Vec score_count_;
  Vec max_entropy_;
  Vec max_entropy_index_;
  int candidates_per_param_;
  double alpha_;
  int num_params_;
  std::atomic<bool> thread_should_run_;
  std::shared_ptr<std::thread> thread_handle_;

  // synchronization variables
  std::condition_variable data_load_cv_;
  std::mutex data_load_mutex_;
  bool data_loaded_flg_;
};

#endif  // ENTROPYTABLE_H__
