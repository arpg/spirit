#ifndef PRIORITYQUEUE_H__
#define PRIORITYQUEUE_H__

#include <spirit/Calibration/CandidateWindow.h>

class PriorityQueue {
public:
  PriorityQueue(unsigned int queue_size):queue_size_(queue_size) {
    queue_max_entropy_index_ = 0;
    entropy_alpha_ = 1;
  }

  ~PriorityQueue(){}

  int PushBackCandidateWindow(const CandidateWindow& candidate) {
    // accept very firt candidate
    if(queue_.size() == 0) {
      queue_.push_back(candidate);
      // update highest entropy value and index in queue
      FindHighestEntropy();
      return queue_.size();
    }

    // ln(0.95)=-0.05129 for 95% safety margin
    if(!(candidate.GetEntropy() < queue_max_entropy_-0.05129)) {
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
      if(candidate.GetEntropy() > queue_[common_index].GetEntropy()) {
        return -1;
      }
      queue_.erase(queue_.begin()+common_index);
    }
    queue_.push_back(candidate);

    // update highest entropy value and index in queue
    FindHighestEntropy();
//    std::cout << "max entropy " << queue_max_entropy_ << std::endl;
    return queue_.size();
  }

  void OptimizeParametersInQueue(spVehicleConstructionInfo& current_params) {
    OptimizationThread(current_params);
  }

  double GetQueueMaxEntropy() {
    return queue_max_entropy_;
  }

private:
  double OptimizationThread(spVehicleConstructionInfo& current_params) {
    Eigen::Vector2d parameter_vec;
    parameter_vec[0] = (current_params.wheels_anchor[0]-current_params.wheels_anchor[1])[1];
    parameter_vec[1] = current_params.wheel_friction;
    ceres::Problem problem;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd residual_weight(17);
    residual_weight << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    for(int ii=0; ii<queue_.size(); ii++) {
      ceres::CostFunction* cost_function = new CalibCostFunc(current_params,queue_[ii].state_series_,residual_weight,jacobian);
      problem.AddResidualBlock(cost_function, NULL, parameter_vec.data());
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
    std::cout << parameter_vec[0] << "," << parameter_vec[1]  << "," << covariance_(1,1) << "," << covariance_(1,1)  << std::endl;

    current_params.wheels_anchor[0][1] = parameter_vec[0]/2;
    current_params.wheels_anchor[1][1] = -parameter_vec[0]/2;
    current_params.wheels_anchor[2][1] = -parameter_vec[0]/2;
    current_params.wheels_anchor[3][1] = parameter_vec[0]/2;
    current_params.wheel_friction = parameter_vec[1];
  }

  void FindHighestEntropy() {
    queue_max_entropy_ = queue_[0].GetEntropy();
    queue_max_entropy_index_ = 0;
    for(int ii=1; ii<queue_.size(); ii++) {
      if(queue_[ii].GetEntropy() > queue_max_entropy_) {
        queue_max_entropy_ = queue_[ii].GetEntropy();
        queue_max_entropy_index_ = ii;
      }
    }
  }

  int FindCommonCandidate(const CandidateWindow& candidate) {
    for(int ii=0; ii<queue_.size(); ii++) {
      // last timestamp of ii'th candidate in queue
      spTimestamp t0 = queue_[ii].state_series_.back()->time_stamp;
      // first timestamp of given candidate
      spTimestamp t1 = candidate.state_series_.front()->time_stamp;
//      if(spGeneralTools::TickTock_ms(t0,t1)<=0) {
      if(t0>t1) {
        return ii;
      }
    }
    return -1;
  }

  unsigned int queue_size_;
  double entropy_alpha_;
  std::vector<CandidateWindow> queue_;
  double queue_max_entropy_;
  int queue_max_entropy_index_;
};

#endif // PRIORITYQUEUE_H__
