#ifndef PRIORITYQUEUE_H__
#define PRIORITYQUEUE_H__

#include <spirit/Calibration/CandidateWindow.h>
#include <condition_variable>
#include <atomic>

class PriorityQueue {
public:
  PriorityQueue(int queue_size, std::shared_ptr<spVehicleConstructionInfo> car_params):queue_size_(queue_size), current_params_(car_params->MakeCopy()) {
    queue_max_entropy_index_ = 0;
    entropy_alpha_ = 0.05129;
    data_loaded_flg_ = false;
    thread_should_run_ = true;
    // run priority queue optimization in another thread
    thread_handle_ = std::make_unique<std::thread>(&PriorityQueue::OptimizeParametersInQueue,this);
  }

  ~PriorityQueue(){
    thread_should_run_ = false;
    thread_handle_->join();
    thread_handle_.reset();
  }

  int PushBackCandidateWindow(std::shared_ptr<CandidateWindow> candidate) {
    // accept very firt candidate
    if(queue_.size() == 0) {
      queue_.push_back(candidate);
      // update highest entropy value and index in queue
      FindHighestEntropy();
      return queue_.size();
    }
    // ln(0.95)=-0.05129 for 95% safety margin
    if(!(candidate->GetEntropy() < queue_max_entropy_-entropy_alpha_)) {
      return -1;
    }
    // check if this candidate has common segment with any candidate in queue
    int common_index = FindCommonCandidate(candidate);
    if(common_index == -1) {
      // if entropy was lower than max entropy an queue is full then replace it with highest entropy
      if(queue_.size() == queue_size_) {
        queue_.erase(queue_.begin()+queue_max_entropy_index_);
      }
    } else {
      // if there is a common segment then replace this with that
      if(candidate->GetEntropy() > queue_[common_index]->GetEntropy()) {
        return -1;
      }
      queue_.erase(queue_.begin()+common_index);
    }
    // push new candiate to queue
    queue_.push_back(candidate);

    // update highest entropy value and index in queue
    FindHighestEntropy();
//    std::cout << "size is " << queue_.size() << std::endl;

    // notify the optimization thread that there is a new data
    {
      std::lock_guard<std::mutex> lock(data_load_mutex_);
      data_loaded_flg_ = true;
      data_load_cv_.notify_one();
    }

    return queue_.size();
  }

  void OptimizeParametersInQueue(/*spVehicleConstructionInfo& current_params*/) {
    while(thread_should_run_) {
      // wait until new candidate window pushed back
      {
        std::unique_lock<std::mutex> lock(data_load_mutex_);
        data_load_cv_.wait(lock,std::bind(&PriorityQueue::GetDataLoadedFlag,this));
        data_loaded_flg_ = false;
      }
      // take a copy of queue_ for optimization. this is to avoid queue modification from other threads while optimizing
      // since we are just reading the queue_ object this should be thred safe
      queue_opt_.clear();
      queue_opt_ = queue_;

      // optimize queue with new candidate windows
      OptimizationThread();
    }
  }

  double GetQueueMaxEntropy() {
    return queue_max_entropy_;
  }

  bool GetParameters(std::shared_ptr<spVehicleConstructionInfo> car_params) {
    if(current_param_mutex_.try_lock()) {
      car_params = current_params_->MakeCopy();
      current_param_mutex_.unlock();
      return true;
    }
    return false;
  }

private:
  double OptimizationThread() {
    ceres::Problem problem;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd residual_weight(17);
    residual_weight << 1,1,1,0.8,0.8,0.8,0,0,0,0,0,0,0,0,0,0,0;
    current_param_mutex_.lock();
    Eigen::VectorXd parameter_vec(current_params_->GetParameterVector());
    for(int ii=0; ii<queue_opt_.size(); ii++) {
      ceres::CostFunction* cost_function = new CalibCostFunc(current_params_,queue_opt_[ii]->state_series_opt_,residual_weight,jacobian);
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), parameter_vec.data());
    }
    ceres::CostFunction* loss_function = new ParamLimitLossFunc<2>(current_params_->calib_min_limit_vec,current_params_->calib_max_limit_vec,100);
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
    std::cout << parameter_vec[0] << "," << parameter_vec[1]  << "," << covariance_(1,1) << "," << covariance_(1,1) << ";" << std::endl;

    current_params_->SetParameterVector(parameter_vec);
    current_param_mutex_.unlock();
    std::cout << "entropies are " ;
    for(int ii=0; ii<queue_.size(); ii++) {
      std::cout << queue_[ii]->GetEntropy() << "," ;
    }
    std::cout << std::endl;
  }

  void FindHighestEntropy() {
    queue_max_entropy_ = queue_[0]->GetEntropy();
    queue_max_entropy_index_ = 0;
    for(int ii=1; ii<queue_.size(); ii++) {
      if(queue_[ii]->GetEntropy() > queue_max_entropy_) {
        queue_max_entropy_ = queue_[ii]->GetEntropy();
        queue_max_entropy_index_ = ii;
      }
    }
  }

  int FindCommonCandidate(std::shared_ptr<CandidateWindow> candidate) {
    for(int ii=0; ii<queue_.size(); ii++) {
      // last timestamp of ii'th candidate in queue
      spTimestamp t0 = queue_[ii]->state_series_opt_.back()->time_stamp;
      // first timestamp of given candidate
      spTimestamp t1 = candidate->state_series_opt_.front()->time_stamp;
      if(t0>=t1) {
        return ii;
      }
    }
    return -1;
  }

  bool GetDataLoadedFlag(){
    return data_loaded_flg_;
  }

  unsigned int queue_size_;
  double entropy_alpha_;
  std::shared_ptr<spVehicleConstructionInfo> current_params_;
  std::mutex current_param_mutex_;
  std::vector<std::shared_ptr<CandidateWindow>> queue_;
  std::vector<std::shared_ptr<CandidateWindow>> queue_opt_;
  double queue_max_entropy_;
  int queue_max_entropy_index_;
  std::shared_ptr<std::thread> thread_handle_;
  std::atomic<bool> thread_should_run_;

  // synchronization variables
  std::condition_variable data_load_cv_;
  std::mutex data_load_mutex_;
  bool data_loaded_flg_;
};

#endif // PRIORITYQUEUE_H__
