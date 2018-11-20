#ifndef LOGPARSER_H__
#define LOGPARSER_H__

#include <fstream>
#include <string>
//#include <Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/CalibDerCarODE.h>


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

  void ReplaceWithSimData(){
    RK4<CalibDerODE> rk4solver(0.01);
    Eigen::VectorXd params(14);
//    params[0] = 4.392;
//    params[1] = 4.392;
//    params[2] = 0.0636;
//    params[3] = 12.69;
//    params[4] = 14.83;
//    params[5] = 84.16;
//    params[6] = 1.024;
//    params[7] = 1.1790;
//    params[8] = 7.07;
//    params[9] = 1.059;
//    params[10] = 1.118;
//    params[11] = 5.72;
//    params[12] = 0.68;
//    params[13] = 4.88;
    params[0] = 4.392;
    params[1] = 4.392;
    params[2] = 0.636;
    params[3] = 1.01;

    params[4] = 0.4;
    params[5] = 0.0001;
    params[6] = 0;
    params[7] = 1;

    params[8] = 0.0001;
    params[9] = 0.0001;
    params[10] = 0;
    params[11] = 1;

    params[12] = 0.01;
    params[13] = 4.9;
    rk4solver.SetParameterVec(params);
    Eigen::MatrixXd iter_jac(11,14);
    Eigen::VectorXd curr_u(2);
    Eigen::VectorXd curr_state(11);
    curr_state = Eigen::VectorXd::Zero(11);
    if(state_input_vec_[1].data_type == 1){
      curr_state[3] = state_input_vec_[1].state[0];
      curr_state[4] = state_input_vec_[1].state[1];
      curr_state[5] = state_input_vec_[1].state[2];
      curr_state[6] = state_input_vec_[1].state[3];
      curr_state[10] = state_input_vec_[1].state[4];
      std::cout << "got type 1"<< std::endl;
    }
    if(state_input_vec_[0].data_type == 2){
      curr_state[7] = state_input_vec_[0].state[0];
      curr_state[8] = state_input_vec_[0].state[1];
      curr_state[9] = state_input_vec_[0].state[2];
      curr_state[0] = state_input_vec_[0].state[3];
      curr_state[1] = state_input_vec_[0].state[4];
      curr_state[2] = state_input_vec_[0].state[5];
      std::cout << "got type 2"<< std::endl;
    }

    if(curr_state[0] == 0){
      curr_state[0] = 0.01;
    }

    double time_diff;

    for(int ii=1; ii<state_input_vec_.size()-1; ii++){
      curr_u[0] = state_input_vec_[ii].input[0];
      curr_u[1] = state_input_vec_[ii].input[1];
      time_diff = state_input_vec_[ii+1].timestamp - state_input_vec_[ii].timestamp;
      rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

      if(state_input_vec_[ii+1].data_type == 1){
        state_input_vec_[ii+1].state[0] = curr_state[3];
        state_input_vec_[ii+1].state[1] = curr_state[4];
        state_input_vec_[ii+1].state[2] = curr_state[5];
        state_input_vec_[ii+1].state[3] = curr_state[6];
        state_input_vec_[ii+1].state[4] = curr_state[10];
      }
      if(state_input_vec_[ii+1].data_type == 2){
        state_input_vec_[ii+1].state[0] = curr_state[7];
        state_input_vec_[ii+1].state[1] = curr_state[8];
        state_input_vec_[ii+1].state[2] = curr_state[9];
        state_input_vec_[ii+1].state[3] = curr_state[0];
        state_input_vec_[ii+1].state[4] = curr_state[1];
        state_input_vec_[ii+1].state[5] = curr_state[2];
      }
    }

    if(state_input_vec_[1].data_type == 1){
      curr_state[3] = state_input_vec_[1].state[0];
      curr_state[4] = state_input_vec_[1].state[1];
      curr_state[5] = state_input_vec_[1].state[2];
      curr_state[6] = state_input_vec_[1].state[3];
      curr_state[10] = state_input_vec_[1].state[4];
      std::cout << "got type 1"<< std::endl;
    }
    if(state_input_vec_[0].data_type == 2){
      curr_state[7] = state_input_vec_[0].state[0];
      curr_state[8] = state_input_vec_[0].state[1];
      curr_state[9] = state_input_vec_[0].state[2];
      curr_state[0] = state_input_vec_[0].state[3];
      curr_state[1] = state_input_vec_[0].state[4];
      curr_state[2] = state_input_vec_[0].state[5];
      std::cout << "got type 2"<< std::endl;
    }

    if(curr_state[0] == 0){
      curr_state[0] = 0.01;
    }
    for(int ii=1; ii<state_input_vec_.size()-1; ii++){
      curr_u[0] = state_input_vec_[ii].input[0];
      curr_u[1] = state_input_vec_[ii].input[1];
      time_diff = state_input_vec_[ii+1].timestamp - state_input_vec_[ii].timestamp;
      rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

      if(state_input_vec_[ii+1].data_type == 1){
        if((state_input_vec_[ii+1].state[0] == curr_state[3])&&
        (state_input_vec_[ii+1].state[1] == curr_state[4])&&
        (state_input_vec_[ii+1].state[2] == curr_state[5])&&
        (state_input_vec_[ii+1].state[3] == curr_state[6])&&
        (state_input_vec_[ii+1].state[4] == curr_state[10])){
          //nothing
        } else{
          std::cout << "err" << (state_input_vec_[ii+1].state[0] - curr_state[3]) << std::endl;
          SPERROREXIT("not matching 1");
        }
      }
      if(state_input_vec_[ii+1].data_type == 2){
        if((state_input_vec_[ii+1].state[0] == curr_state[7])&&
        (state_input_vec_[ii+1].state[1] == curr_state[8])&&
        (state_input_vec_[ii+1].state[2] == curr_state[9])&&
        (state_input_vec_[ii+1].state[3] == curr_state[0])&&
        (state_input_vec_[ii+1].state[4] == curr_state[1])&&
        (state_input_vec_[ii+1].state[5] == curr_state[2])){
          // nothing
        } else {
          SPERROREXIT("not matching 2");
        }


      }
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
