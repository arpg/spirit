#ifndef PRIORITYQUEUE_H__
#define PRIORITYQUEUE_H__

#include <spirit/Calibration/CandidateWindow.h>

class PriorityQueue {
public:
  PriorityQueue(unsigned int queue_size):queue_size_(queue_size) {
    queue_max_entropy_ = 0;
    queue_max_entropy_index_ = 0;
  }

  ~PriorityQueue(){}

  int PushBackCandidateWindow(std::shared_ptr<CandidateWindow> candidate) {
    // if queue is not full then just pushback
    if(queue_.size() < queue_size_) {
      queue_.push_back(candidate);
      return queue_.size();
    }
    // update highest entropy value and index in queue
    FindHighestEntropy();
    // if queue is full then check the entropies
    if(candidate->GetEntropy() >= queue_max_entropy_){
      return -1;
    }
    // if entropy was lower than max entropy then replace it with highest entropy
    if(queue_.size() == queue_size_) {
      queue_.erase(queue_.begin()+queue_max_entropy_index_);
    }
    queue_.push_back(candidate);
    return queue_.size();
  }

  void OptimizeParametersInQueue() {}

  double GetQueueMaxEntropy() {
    return queue_max_entropy_;
  }

private:
  void FindHighestEntropy() {
    for(int ii=0; ii<queue_.size(); ii++) {
      if(queue_[ii]->GetEntropy() > queue_max_entropy_) {
        queue_max_entropy_ = queue_[ii]->GetEntropy();
        queue_max_entropy_index_ = ii;
      }
    }
  }

  unsigned int queue_size_;
  std::vector<std::shared_ptr<CandidateWindow>> queue_;
  double queue_max_entropy_;
  int queue_max_entropy_index_;
};

#endif // PRIORITYQUEUE_H__
