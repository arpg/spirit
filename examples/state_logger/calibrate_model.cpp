
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include "LogParser.h"
#include "AnalyticalCalibCostFunc.h"

int main(){

  // read dataset and store in variables
  LogParser log("/Users/saghli/code/remotelog/log8.csv");

  if(!log.ParseCsvLog()){
    std::cerr << "Error while parsing the log file. Exiting ..." << std::endl;
    return -1;
  }

  log.ComputeStateInputVec();
  log.ReplaceWithSimData();

//  for(int ii=0; ii<log.state_input_vec_.size();ii++){
//    if(log.state_input_vec_[ii].data_type==2){
//      std::cout << log.state_input_vec_[ii].state[3] << " , " << log.state_input_vec_[ii].state[4] << std::endl;
//    }
//  }

  ///////////////////////////////////////////////// Testing trajectory

//  RK4<CalibDerODE> rk4solver(0.01);
//  Eigen::VectorXd parameters1(14);
////  params[0] = 12;
////  params[1] = 12;
////  params[2] = 0.042;
////  params[3] = 12.69;
////  params[4] = 14.83;
////  params[5] = 84.16;
////  params[6] = 0.024;
////  params[7] = 0.1790;
////  params[8] = 7.07;
////  params[9] = 0.059;
////  params[10] = 0.118;
////  params[11] = 5.72;
////  params[12] = 0.68;
////  params[13] = 4.88;

//  parameters1[0] = 4.392;
//  parameters1[1] = 4.392;
//  parameters1[2] = 0.0636;
//  parameters1[3] = 0.001;

//  parameters1[4] = 0.001;
//  parameters1[5] = 0.001;
//  parameters1[6] = 1;
//  parameters1[7] = 0;

//  parameters1[8] = 150.1;
//  parameters1[9] = 0.1;
//  parameters1[10] = 1;
//  parameters1[11] = 0;

//  parameters1[12] = 0.01;
//  parameters1[13] = 0.1;
//  rk4solver.SetParameterVec(parameters1);
//  Eigen::MatrixXd iter_jac(11,14);
//  Eigen::VectorXd curr_u(2);
//  Eigen::VectorXd curr_state(11);
//  curr_state = Eigen::VectorXd::Zero(11);
//  if(log.state_input_vec_[1].data_type == 1){
//    curr_state[3] = log.state_input_vec_[1].state[0];
//    curr_state[4] = log.state_input_vec_[1].state[1];
//    curr_state[5] = log.state_input_vec_[1].state[2];
//    curr_state[6] = log.state_input_vec_[1].state[3];
//    curr_state[10] = log.state_input_vec_[1].state[4];
//  }
//  if(log.state_input_vec_[0].data_type == 2){
//    curr_state[7] = log.state_input_vec_[0].state[0];
//    curr_state[8] = log.state_input_vec_[0].state[1];
//    curr_state[9] = log.state_input_vec_[0].state[2];
//    curr_state[0] = log.state_input_vec_[0].state[3];
//    curr_state[1] = log.state_input_vec_[0].state[4];
//    curr_state[2] = log.state_input_vec_[0].state[5];
//    std::cout << "vx " << curr_state[0] << std::endl;
//    std::cout << "vy " << curr_state[1] << std::endl;
//  }

//  if(curr_state[0] == 0){
//    curr_state[0] = 0.01;
//  }

//  double time_diff;
//  std::ofstream myfile;

//  myfile.open("path.csv",std::ofstream::trunc);

//  for(int ii=0; ii<10000/*log.state_input_vec_.size()-1*/; ii++){
////    curr_u[0] = log.state_input_vec_[ii].input[0];
////    curr_u[1] = log.state_input_vec_[ii].input[1];
//    curr_u[0] = 0.7;
//    curr_u[1] = 1;
//    time_diff = 0.01;//log.state_input_vec_[ii+1].timestamp - log.state_input_vec_[ii].timestamp;
//    rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

//    double linvel_x = std::cos(curr_state[9])*curr_state[0] - std::sin(curr_state[9])*curr_state[1];
//    double linvel_y = std::sin(curr_state[9])*curr_state[0] + std::cos(curr_state[9])*curr_state[1];
//    for(int ii=0; ii<11; ii++){
//      if(ii==0){
//        myfile << linvel_x << ",";
//      } else if(ii==1){
//        myfile << linvel_y << ",";
////      } else if(ii==7){
////        myfile << pose.translation()[0] << ",";
////      } else if(ii==8){
////        myfile << pose.translation()[1] << ",";
//      } else {
//        myfile << curr_state[ii] << ",";
//      }
//    }
//    myfile << "\n";


//  }
//  myfile.close();

//  SPERROREXIT("done testing");
  //////////////////////////////////////////////////////////////

  // construct ceres cost function
  const int num_params = 14;

  // create sqrt of weighting vector
  Eigen::VectorXd weights(11);
//  weights << 0.9,0.9,0.9,0.1,0.1,0.1,0.1,1,1,1,0;
  weights << 1,1,1,1,1,1,1,1,1,1,1;

  ceres::Problem problem;
  ceres::CostFunction* calib_cost = new AnalyticalCalibCostFunc(log,num_params,weights);

  Eigen::VectorXd min_limits(14);
  Eigen::VectorXd max_limits(14);
  for(int ii=0; ii<14; ii++) {
    min_limits[ii] = 0;
    max_limits[ii] = 100;
  }

//  min_limits[4] = -10;
//  min_limits[5] = -10;
//  min_limits[8] = -10;
//  min_limits[9] = -10;


//  ceres::CostFunction* loss_function = new LogBaarrierLossFunc<14>(min_limits,max_limits,0.1);
  ceres::CostFunction* loss_function = new ParamLimitLossFunc<14>(min_limits,max_limits,1000);
  double parameters[num_params];

  parameters[0] = 4.392;
  parameters[1] = 4.392;
  parameters[2] = 0.636;
  parameters[3] = 0.51;

  parameters[4] = 0.4;
  parameters[5] = 0.0001;
  parameters[6] = 0;
  parameters[7] = 1;

  parameters[8] = 0.0001;
  parameters[9] = 0.0001;
  parameters[10] = 0;
  parameters[11] = 1;

  parameters[12] = 0.01;
  parameters[13] = 4.9;

//  parameters[0] = 4.392;
//  parameters[1] = 4.392;
//  parameters[2] = 0.17;
//  parameters[3] = 0.1;

//  parameters[4] = 10;
//  parameters[5] = 0.001;
//  parameters[6] = 1;
//  parameters[7] = 0;

//  parameters[8] = 15.1;
//  parameters[9] = 0.1;
//  parameters[10] = 1;
//  parameters[11] = 0;

//  parameters[12] = 0.01;
//  parameters[13] = 5.1;

//  problem.AddResidualBlock(calib_cost,new ceres::TukeyLoss(1000), parameters);
  problem.AddResidualBlock(calib_cost,NULL, parameters);
//  problem.AddResidualBlock(loss_function,NULL,parameters);

  std::vector<int> fix_param_vec;
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);

//  fix_param_vec.push_back(2);
//  fix_param_vec.push_back(3);

//  fix_param_vec.push_back(4);
//  fix_param_vec.push_back(5);
//  fix_param_vec.push_back(6);
//  fix_param_vec.push_back(7);

//  fix_param_vec.push_back(8);
//  fix_param_vec.push_back(9);
//  fix_param_vec.push_back(10);
//  fix_param_vec.push_back(11);

  fix_param_vec.push_back(12);

//    fix_param_vec.push_back(13);
  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(14,fix_param_vec);
  problem.SetParameterization(parameters,subparam);

//  ceres::Problem::EvaluateOptions opt;
//  double cost = 0;
//  for (int ii = 0; ii < 100; ++ii) {
//    parameters[12] = 0.1+1*ii/99.0;
//    problem.Evaluate(opt,&cost,nullptr,nullptr,nullptr);
//    std::cout << parameters[12] << ", " << cost << std::endl;
//  }
//SPERROREXIT("done");

  // Run the solver!
  ceres::Solver::Options options;
  options.initial_trust_region_radius = 0.1;
//  options.max_trust_region_radius = 100000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//  options.max_consecutive_nonmonotonic_steps = 10;
//  options.max_num_consecutive_invalid_steps = 2;
  options.min_relative_decrease = 1e-4;
//  options.parameter_tolerance = 1e-3;
//  options.max_trust_region_radius = 0.1;
  options.function_tolerance = 1e-8;
//  options.gradient_tolerance = 1e-4;
//  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = true;
  options.gradient_check_numeric_derivative_relative_step_size = 0.1;
  options.gradient_check_relative_precision = 1e-3;
  options.check_gradients = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  std::cout << "solution is ->\n";
  for (int ii = 0; ii < num_params; ++ii) {
    std::cout << parameters[ii] << " , ";
  }
  std::cout << std::endl;


  return 0;
}
