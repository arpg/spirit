#ifndef CANDIDATEWINDOW_H__
#define CANDIDATEWINDOW_H__

#include <spirit/Types/spTypes.h>

class CandidateWindow {
 public:
  CandidateWindow(unsigned int window_size) : window_size_(window_size){
  }

  ~CandidateWindow(){}

  int PushBackState(const spState& state) {
    if(!CheckPossibleCandidate(state)) {
      return -1;
    }
    if(state_series_.size() == window_size_) {
      state_series_.erase(state_series_.begin());
    }
    state_series_.push_back(std::make_shared<spState>(state));
    return state_series_.size();
  }

  void CalculateEntropy(){}
  void OptimizeParametersInWindow(){}
  double GetEntropy(){
    return entroy_;
  }
  // check if this state could be added to candidate window
  bool CheckPossibleCandidate(const spState& state) {
    // accept everything for now. TODO(sina) later I should reject entries which add null spaces to the optimization problem.
    return true;
  }


 private:

  unsigned int window_size_;
  double entroy_;
  Eigen::MatrixXd covariance_;
  spStateSeries state_series_;
};

#endif //CANDIDATEWINDOW_H__
