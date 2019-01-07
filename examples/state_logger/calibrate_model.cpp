
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include "LogParser.h"
#include "AnalyticalCalibCostFunc.h"

int main(){

  // read dataset and store in variables
  LogParser log("/Users/saghli/code/remotelog/log2.csv");

  if(!log.ParseCsvLog()){
    std::cerr << "Error while parsing the log file. Exiting ..." << std::endl;
    return -1;
  }

  log.ComputeStateInputVec();
//  log.ReplaceWithSimData();

//  for(int ii=0; ii<log.state_input_vec_.size();ii++){
//    if(log.state_input_vec_[ii].data_type==2){
//      std::cout << log.state_input_vec_[ii].state[3] << " , " << log.state_input_vec_[ii].state[4] << std::endl;
//    }
//  }

  // construct ceres cost function
  const int num_params = 14;

  // create sqrt of weighting vector
  Eigen::VectorXd weights(11);
//  weights << 0,0,0,0,0,0,0,1,1,0,0;
  weights << 0.1,0.1,0.1,100,100,100,100,0.1,0.1,100,0.1;
//  weights << 100,100,100,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1;
//  weights << 1,1,1,1,1,1,1,1,1,1,1;

  ceres::Problem problem;
  ceres::CostFunction* calib_cost = new AnalyticalCalibCostFunc(log,num_params,weights);

  Eigen::VectorXd min_limits(14);
  Eigen::VectorXd max_limits(14);
  for(int ii=0; ii<14; ii++) {
    min_limits[ii] = 0;
    max_limits[ii] = 1e+5;
  }

  min_limits[5] = -1e+5;
  min_limits[6] = -1e+5;
  min_limits[7] = -1e+5;
  min_limits[9] = -1e+5;
  min_limits[10] = -1e+5;
  min_limits[11] = -1e+5;


//  ceres::CostFunction* loss_function = new LogBaarrierLossFunc<14>(min_limits,max_limits,0.1);
  ceres::CostFunction* loss_function = new ParamLimitLossFunc<14>(min_limits,max_limits,100);
  double parameters[num_params];

  //4.3 , 4.3 , 2.22989 , 7.90739e-05 , 0.757066 , 0 , 1 , 0 , 25.9108 , 0 , 1 , 0 , 0.01 , 0.04659 ,
  parameters[0] = 4.3;
  parameters[1] = 4.3;
  parameters[2] = 2.016;
  parameters[3] = 0.001;

  parameters[4] = 0.7;
  parameters[5] = 0;
  parameters[6] = 1;
  parameters[7] = 0;

  parameters[8] = 25;
  parameters[9] = 0;
  parameters[10] = 1;
  parameters[11] = 0;

  parameters[12] = 0.01;
  parameters[13] = 0.04;

//  problem.AddResidualBlock(calib_cost,new ceres::TukeyLoss(100000), parameters);
  problem.AddResidualBlock(calib_cost,NULL, parameters);
//  problem.AddResidualBlock(loss_function,NULL,parameters);

  std::vector<int> fix_param_vec;
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);

//  fix_param_vec.push_back(2);
//  fix_param_vec.push_back(3);

//  fix_param_vec.push_back(4);
  fix_param_vec.push_back(5);
  fix_param_vec.push_back(6);
  fix_param_vec.push_back(7);

//  fix_param_vec.push_back(8);
  fix_param_vec.push_back(9);
  fix_param_vec.push_back(10);
  fix_param_vec.push_back(11);

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
  options.initial_trust_region_radius = 0.01;
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
  options.max_num_iterations = 500;
  options.minimizer_progress_to_stdout = true;
  options.gradient_check_numeric_derivative_relative_step_size = 0.0001;
  options.gradient_check_relative_precision = 1e-1;
//  options.check_gradients = true;
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
