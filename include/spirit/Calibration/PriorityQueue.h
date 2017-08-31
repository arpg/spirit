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
      std::cout << "added to position " << queue_.size() << std::endl;
      return queue_.size();
    }

    if(candidate.GetEntropy() > entropy_alpha_*queue_max_entropy_){
      return -1;
    }
    // check if this candidate has common segment with any candidate in queue
    int common_index = FindCommonCandidate(candidate);
    if(common_index == -1) {
      // if entropy was lower than max entropy an queue is full then replace it with highest entropy
      if(queue_.size() == queue_size_) {
        std::cout << "removing position " << queue_max_entropy_index_ << std::endl;
        queue_.erase(queue_.begin()+queue_max_entropy_index_);
      }
    } else {
      // if there is a common segment then replace this with that
      if(candidate.GetEntropy() > queue_[common_index].GetEntropy()) {
        return -1;
      }
      std::cout << "removing position " << queue_max_entropy_index_ << std::endl;
      queue_.erase(queue_.begin()+common_index);
    }
    queue_.push_back(candidate);

    // update highest entropy value and index in queue
    FindHighestEntropy();

    std::cout << "added to position " << queue_.size() << std::endl;
    return queue_.size();
  }

  void OptimizeParametersInQueue() {}

  double GetQueueMaxEntropy() {
    return queue_max_entropy_;
  }

private:
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
