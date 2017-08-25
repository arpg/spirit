#include <spirit/Controllers/spMPC.h>

spMPC::spMPC(const spVehicleConstructionInfo& car_params, float horizon_duration): car_params_(car_params) {
  horizon_ = (int)(horizon_duration/DISCRETIZATION_STEP_SIZE);
}

spMPC::~spMPC() {

}

int spMPC::CalculateControls(const spTrajectory& ref_traj, const spState& curr_state, spCtrlPts2ord_2dof& controls) {
  // find the closest trajectory point to current state
  int closest_index;
  int closest_subindex;
  FindClosestTrajPoint(closest_index,closest_subindex,ref_traj,curr_state);
  // crop trajectory from closest traj point up to horizon
  spStateSeries ref_states;
  int index = closest_index;
  // skip to next subindex so horizon starts from next subindex
  int subindex = closest_subindex+1;
  for(int ii=0; ii<horizon_; ii++) {
    // skip the last subindex since its same as subindex0 of next index
    if(subindex < (*ref_traj.GetTrajectoryStateSeries(index)).size()-1) {
      ref_states.push_back((*ref_traj.GetTrajectoryStateSeries(index))[subindex]);
    } else {
      // get the next waypoint index
      if(index == ref_traj.GetNumWaypoints()-1) {
        if(ref_traj.IsLoop()) {
          index = 0;
        } else {
          SPERROR("Reached end of the trajectory, no further controls can be calculated.");
          return 0;
        }
      } else {
        index++;
      }
      subindex = 0;
      ref_states.push_back((*ref_traj.GetTrajectoryStateSeries(index))[subindex]);
    }
    subindex++;
  }

  // construct and minimize the MPC cost function, then return controls found
  MinimizeMPCError(ref_states,curr_state,controls);
  return 1;
}

void spMPC::SetHorizon(float horizon_duration) {
  horizon_ = (int)(horizon_duration/DISCRETIZATION_STEP_SIZE);
}


// Brutefoce search for closest state of reference trajectory to current state
// This could be replaced with a faster search algorithm later
void spMPC::FindClosestTrajPoint(int& closest_waypoint_index, int& closest_subindex, const spTrajectory& ref_traj, const spState& curr_state) {
  double smallest_norm;
  spState state_diff;
  for(int ii=0; ii<ref_traj.GetNumWaypoints(); ii++) {
    if(ref_traj.GetTravelDuration(ii) == -1) {
      SPERROREXIT("LocalPlanner solution doesn't exist");
    }
    // skip the last subindex since its same as subindex0 of next index
    for(int jj=0; jj<ref_traj.GetTrajectoryStateSeries(ii)->size()-1; jj++) {
      // calculate the distance of current state to every trajectory state
      state_diff = curr_state - *((*ref_traj.GetTrajectoryStateSeries(ii))[jj]);
      // only check for xyz difference norm
      double curr_norm = state_diff.pose.translation().squaredNorm();
      if((smallest_norm > curr_norm) || (jj+ii == 0)) {
        smallest_norm = curr_norm;
        closest_subindex = jj;
        closest_waypoint_index = ii;
      }
    }
  }
}

void spMPC::MinimizeMPCError(const spStateSeries& ref_states,const spState& current_state, spCtrlPts2ord_2dof& controls) {
  ceres::Problem problem;
  Eigen::VectorXd residual_weight(12);
//  residual_weight << 4, 4, 4, 3, 3, 3, 0.07, 0.07, 0.07, 0.1, 0.1, 0.1;
//  residual_weight << 1,1,1,0.5,0.5,0.5,0.1,0.1,0.1,0,0,0;
  residual_weight << 1,1,1,0.1,0.1,0.1,0.001,0.001,0.001,0.001,0.001,0.001;
  Eigen::VectorXd traj_point_weight(horizon_);
  traj_point_weight.setOnes(horizon_);
//  traj_point_weight[0] = 0.5;
//  traj_point_weight[1] = 0.8;
  // put more weight on trajecotry point errors rather than residula weights
  traj_point_weight = 10*traj_point_weight;
  ceres::CostFunction* cost_function = new MPCCostFunc(car_params_,current_state,ref_states,residual_weight,traj_point_weight);
  Eigen::VectorXd min_limits(6);
  Eigen::VectorXd max_limits(6);
  for(int ii=0; ii<6; ii+=2) {
    min_limits[ii] = -SP_PI_QUART;
    max_limits[ii] = SP_PI_QUART;
  }
  for(int ii=1; ii<6; ii+=2) {
    min_limits[ii] = -200;
    max_limits[ii] = 200;
  }
  ceres::CostFunction* loss_function = new ParamLimitLossFunc<6>(min_limits,max_limits,100);

  double parameters[6];
  for (int ii = 0; ii < 6; ++ii) {
    parameters[ii] = controls.data()[ii];
  }
  problem.AddResidualBlock(cost_function, NULL, parameters);
  problem.AddResidualBlock(loss_function,NULL,parameters);

  std::vector<int> fix_param_vec;
  // fix the first two parameters
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);
//  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(6,fix_param_vec);
//  problem.SetParameterization(parameters,subparam);
//  problem.SetParameterLowerBound(parameters,0,-((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterUpperBound(parameters,0,((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterLowerBound(parameters,1,-200);
//  problem.SetParameterUpperBound(parameters,1,200);
//  problem.SetParameterLowerBound(parameters,2,-((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterUpperBound(parameters,2,((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterLowerBound(parameters,3,-200);
//  problem.SetParameterUpperBound(parameters,3,200);
//  problem.SetParameterLowerBound(parameters,4,-((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterUpperBound(parameters,4,((SP_PI/4)-FINITE_DIFF_EPSILON));
//  problem.SetParameterLowerBound(parameters,5,-200);
//  problem.SetParameterUpperBound(parameters,5,200);



  // Run the solver!
  ceres::Solver::Options options;
//  options.update_state_every_iteration = true;
//  SoverIterationCallback iteration_callback(nullptr,nullptr);
//  options.callbacks.push_back(&iteration_callback);
//    options.check_gradients = true;
//    options.gradient_check_numeric_derivative_relative_step_size = 0.01;
//  options.max_num_iterations = 10;
//  options.min_relative_decrease = 1e-2;
//  options.max_solver_time_in_seconds = 0.1;
  options.initial_trust_region_radius = 0.7;
  options.max_trust_region_radius = 0.7;
//  options.min_trust_region_radius = 1e-32;
  options.parameter_tolerance = 1e-2;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
//  std::cout << summary.FullReport() << std::endl;

  // calculate final Covariance
  /*
  ceres::Covariance::Options cov_options;
  cov_options.num_threads = 1;
  ceres::Covariance covariance(cov_options);
  std::vector<const double*> covariance_blocks;
  covariance_blocks.push_back(parameters);
  CHECK(covariance.Compute(covariance_blocks, &problem));
  double covariance_xx[6 * 6];
  covariance.GetCovarianceBlock(parameters, parameters, covariance_xx);
  std::cout << "covariance is "  << std::endl;
  for(int ii=0;ii<6;ii++) {
    for(int jj=0;jj<6;jj++) {
      std::cout << "\t" << covariance_xx[ii*6+jj] ;
    }
    std::cout  << std::endl;
  }
  std::cout << std::endl;
  */

  for (int ii = 0; ii < 6; ++ii) {
    controls.data()[ii] = parameters[ii];
  }

//  std::cout << "MPCcontrols: \n" << controls << std::endl;

}
