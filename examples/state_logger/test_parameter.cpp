
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include "LogParser.h"
#include "AnalyticalCalibCostFunc.h"

double eval(LogParser& log, Eigen::VectorXd& p){
  RK4<CalibDerODE> rk4solver(0.01);
  rk4solver.SetParameterVec(p);
  Eigen::MatrixXd iter_jac(11,14);
  Eigen::VectorXd curr_u(2);
  Eigen::VectorXd curr_state(11);
  curr_state = Eigen::VectorXd::Zero(11);
  if(log.state_input_vec_[1].data_type == 1){
    curr_state[3] = log.state_input_vec_[1].state[0];
    curr_state[4] = log.state_input_vec_[1].state[1];
    curr_state[5] = log.state_input_vec_[1].state[2];
    curr_state[6] = log.state_input_vec_[1].state[3];
    curr_state[10] = log.state_input_vec_[1].state[4];
  }
  if(log.state_input_vec_[0].data_type == 2){
    curr_state[7] = log.state_input_vec_[0].state[0];
    curr_state[8] = log.state_input_vec_[0].state[1];
    curr_state[9] = log.state_input_vec_[0].state[2];
    curr_state[0] = log.state_input_vec_[0].state[3];
    curr_state[1] = log.state_input_vec_[0].state[4];
    curr_state[2] = log.state_input_vec_[0].state[5];
//    std::cout << "vx " << curr_state[0] << std::endl;
//    std::cout << "vy " << curr_state[1] << std::endl;
  }

  if(curr_state[0] == 0){
    curr_state[0] = 0.01;
  }

  double time_diff;
//  std::ofstream myfile;

  Eigen::VectorXd res(log.GetNumMeaseurements()-5);
  int row_count = 0;

//  myfile.open("path.csv",std::ofstream::trunc);

//  for(int ii=0; ii<10000; ii++){
  for(int ii=0; ii<log.state_input_vec_.size()-1; ii++){
    curr_u[0] = log.state_input_vec_[ii].input[0];
    curr_u[1] = log.state_input_vec_[ii].input[1];
//    curr_u[0] = 0.7;
//    curr_u[1] = 1;
    time_diff = log.state_input_vec_[ii+1].timestamp - log.state_input_vec_[ii].timestamp;//0.01;
    rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

//    double linvel_x = std::cos(curr_state[9])*curr_state[0] - std::sin(curr_state[9])*curr_state[1];
//    double linvel_y = std::sin(curr_state[9])*curr_state[0] + std::cos(curr_state[9])*curr_state[1];
//    for(int ii=0; ii<11; ii++){
//      if(ii==0){
//        myfile << linvel_x << ",";
//      } else if(ii==1){
//        myfile << linvel_y << ",";
//      } else {
//        myfile << curr_state[ii] << ",";
//      }
//    }
//    myfile << "\n";

    if((curr_state[0] != curr_state[0])||
       (curr_state[1] != curr_state[1])||
       (curr_state[2] != curr_state[2])||
       (curr_state[3] != curr_state[3])||
       (curr_state[4] != curr_state[4])||
       (curr_state[5] != curr_state[5])||
       (curr_state[6] != curr_state[6])||
       (curr_state[7] != curr_state[7])||
       (curr_state[8] != curr_state[8])||
       (curr_state[9] != curr_state[9])||
       (curr_state[10] != curr_state[10])){
      return -1;
    }

    if(log.state_input_vec_[ii+1].data_type == 1){
      res[row_count] = curr_state[3] - log.state_input_vec_[ii+1].state[0];
//      res[row_count] *= 0.01;
      row_count++;
      res[row_count] = curr_state[4] - log.state_input_vec_[ii+1].state[1];
//      res[row_count] *= 0.01;
      row_count++;
      res[row_count] = curr_state[5] - log.state_input_vec_[ii+1].state[2];
//      res[row_count] *= 0.01;
      row_count++;
      res[row_count] = curr_state[6] - log.state_input_vec_[ii+1].state[3];
//      res[row_count] *= 0.01;
      row_count++;
      res[row_count] = curr_state[10] - log.state_input_vec_[ii+1].state[4];
      row_count++;
    }

    if(log.state_input_vec_[ii+1].data_type == 2){
      res[row_count] = curr_state[7] - log.state_input_vec_[ii+1].state[0];
      res[row_count] *= 0.10;
      row_count++;
      res[row_count] = curr_state[8] - log.state_input_vec_[ii+1].state[1];
      res[row_count] *= 0.10;
      row_count++;
      res[row_count] = curr_state[9] - log.state_input_vec_[ii+1].state[2];
      double chi_mod = std::fmod(res[row_count]+SP_PI,2*SP_PI);
      if(chi_mod<0){
        chi_mod += 2*SP_PI;
      }
      res[row_count] = chi_mod - SP_PI;
      res[row_count] *= 0.10;
      row_count++;

      res[row_count] = curr_state[0] - log.state_input_vec_[ii+1].state[3];
      row_count++;
      res[row_count] = curr_state[1] - log.state_input_vec_[ii+1].state[4];
      row_count++;
      res[row_count] = curr_state[2] - log.state_input_vec_[ii+1].state[5];
      row_count++;
    }
  }
//  myfile.close();

  return res.squaredNorm();

}

int main(){

  // read dataset and store in variables
  LogParser log("/Users/saghli/code/remotelog/log2.csv");

  if(!log.ParseCsvLog()){
    std::cerr << "Error while parsing the log file. Exiting ..." << std::endl;
    return -1;
  }

  log.ComputeStateInputVec();
//  log.ReplaceWithSimData();

  Eigen::VectorXd parameters(14);

  parameters[0] = 4.3;
  parameters[1] = 4.3;
  parameters[2] = 0.1;
  parameters[3] = 0.1;

  parameters[4] = 0.1;
  parameters[5] = 0;
  parameters[6] = 1;
  parameters[7] = 0;

  parameters[8] = 0.1;
  parameters[9] = 0;
  parameters[10] = 1;
  parameters[11] = 0;

  parameters[12] = 0.01;
  parameters[13] = 0.001;

  int resolution = 5;
  Eigen::VectorXd psol(14);
  Eigen::VectorXd pmax(14);
  pmax[2] = 4;
  pmax[3] = 1;
  pmax[4] = 2;
  pmax[8] = 40;
  pmax[13] = 0.1;


  double min_cost = 1000000000000000000;

  for(int ii0=1; ii0<resolution; ii0++){
  for(int ii1=1; ii1<resolution; ii1++){
  for(int ii2=1; ii2<resolution; ii2++){
  for(int ii3=1; ii3<resolution; ii3++){
  for(int ii4=1; ii4<resolution; ii4++){
    parameters[2] = (ii0/(double)resolution)*pmax[2];
    parameters[3] = (ii1/(double)resolution)*pmax[3];
    parameters[4] = (ii2/(double)resolution)*pmax[4];
    parameters[8] = (ii3/(double)resolution)*pmax[8];
    parameters[13] = (ii4/(double)resolution)*pmax[13];
    double curr_cost = eval(log,parameters);
//    std::cout << parameters.transpose() << std::endl;

    if((curr_cost<min_cost)&&(curr_cost!=-1)){
      min_cost = curr_cost;
      psol = parameters;
      std::cout << parameters.transpose() << std::endl;
      std::cout << "cost is " << curr_cost << std::endl;
    } else {
//      std::cout << "skip" << std::endl;
    }

  }}}}}
  std::cout << "Min cost is " << min_cost << std::endl;
  std::cout << "parameters \n" << psol.transpose() << std::endl;
  return 0;
}
