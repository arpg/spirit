#ifndef LOGPARSER_H__
#define LOGPARSER_H__

#include <fstream>
#include <string>


struct LogSample{
  int data_type = -1;
  double timestamp = -1;
  std::vector<double> data;
};

struct StateInputData{
    int data_type = -1;
    double timestamp = -1;
    std::vector<double> state;
    std::vector<double> input;
};

class LogParser{
public:
  LogParser(std::string file_name) : filename_(file_name){
    num_measurements_ = 0;
    num_controls_ = 0;
  }

  std::vector<LogSample> logvec_;
  std::vector<StateInputData> state_input_vec_;

  unsigned int GetNumMeaseurements() const {
    return num_measurements_;
  }

  unsigned int GetNumControls() const {
    return num_controls_;
  }

  void ComputeStateInputVec(){
    std::cout << "computing vectors " << std::endl;
    // it is assumed that input signal is constant till next update
    StateInputData curr_data;
    bool skip_flag = true;
    for(int ii=0; ii<logvec_.size(); ii++){
      if(logvec_[ii].data_type == 0){
        curr_data.input = logvec_[ii].data;
        curr_data.input[0] *= 0.7853;
        skip_flag = false;
      } else if(skip_flag != true) {
        curr_data.state = logvec_[ii].data;
        curr_data.timestamp = logvec_[ii].timestamp;
        curr_data.data_type = logvec_[ii].data_type;
        state_input_vec_.push_back(curr_data);
      } else {
        if(logvec_[ii].data_type == 1){
          num_measurements_ -= 5;
        } else if(logvec_[ii].data_type == 2){
          num_measurements_ -= 6;
        }
      }
    }

    // dont count first data point
    if(state_input_vec_[0].data_type == 1){
      num_measurements_ -= 5;
    } else if(state_input_vec_[0].data_type == 2){
      num_measurements_ -= 6;
    }
  }

  bool ParseCsvLog(){
    std::cout << "parsing the file " << std::endl;
    std::ifstream ifile(filename_);
    if(!ifile.is_open()){
      std::cerr << "Failed to open file named " << filename_ << std::endl;
    }
    std::string line = "";
    unsigned int linecount = 0;
    while(std::getline(ifile,line)){
      LogSample sample;
      std::istringstream line_iss(line);
      std::string word = "";
      std::getline(line_iss,word,',');
      sample.data_type = std::stoi(word);
      if(sample.data_type == 0){
        num_controls_++;
      } else if(sample.data_type == 1){
        num_measurements_ += 5;
      } else if(sample.data_type == 2){
        num_measurements_ += 6;
      } else {
        std::cerr << "UNKNOWN DATA TYPE CAPTURED AT LINE : " << linecount << std::endl;
        return false;
      }
      std::getline(line_iss,word,',');
      sample.timestamp = std::stod(word);
      while(std::getline(line_iss,word,',')){
        sample.data.push_back(std::stod(word));
      }

      // convert global frame velocity to local frame. TODO:this should be done when capturing data
      if(sample.data_type == 2){
        double chi = sample.data[2];
        double vx = std::cos(-chi)*sample.data[3] - std::sin(-chi)*sample.data[4];
        double vy = std::sin(-chi)*sample.data[3] + std::cos(-chi)*sample.data[4];
        sample.data[3] = vx;
        sample.data[4] = vy;
      }if(sample.data_type == 1){
        sample.data[4] /= 1.91;
      }

      logvec_.push_back(sample);
      linecount++;
    }
    ifile.close();
    return true;
  }

private:

  std::string filename_;
  unsigned int num_measurements_;
  unsigned int num_controls_;

};

#endif
