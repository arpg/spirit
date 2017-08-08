#include <spirit/Controllers/spMPC.h>

spMPC::spMPC(const spVehicleConstructionInfo& car_params, float horizon_duration): car_params_(car_params) {
  horizon_ = (int)(horizon_duration/DISCRETIZATION_STEP_SIZE);
}

spMPC::~spMPC() {

}

void spMPC::CalculateControls(const spTrajectory& ref_traj, const spState& curr_state, spCtrlPts2ord_2dof& controls) {
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
    if(subindex < (*ref_traj.GetTrajectoryStateSeries(index)).size()-1) {
      ref_states.push_back((*ref_traj.GetTrajectoryStateSeries(index))[subindex]);
    } else {
      // get the next waypoint index
      if(index == ref_traj.GetNumWaypoints()-1) {
        if(ref_traj.IsLoop()) {
          index = 0;
        } else {
          SPERROR("Reached end of the trajectory, no further controls can be calculated.");
          return;
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
}

void spMPC::SetHorizon(float horizon_duration) {
  horizon_ = horizon_duration;
}


// Brutefoce search for closest state of reference trajectory to current state
void spMPC::FindClosestTrajPoint(int& closest_waypoint_index, int& closest_subindex, const spTrajectory& ref_traj, const spState& curr_state) {
  double smallest_norm;
  spState state_diff;
  for(int ii=0; ii<ref_traj.GetNumWaypoints()-1; ii++) {
    if(ref_traj.GetTravelDuration(ii) == -1) {
      SPERROREXIT("LocalPlanner solution doesn't exist");
    }
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
  residual_weight << 1,1,1,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001;
  Eigen::VectorXd traj_point_weight(5);
  traj_point_weight << 0.5, 0.7, 1, 1, 1, 1, 1, 1, 1, 1;
  traj_point_weight = 10*traj_point_weight;
  ceres::CostFunction* cost_function = new MPCCostFunc(car_params_,current_state,ref_states,residual_weight,traj_point_weight);
//  ceres::CostFunction* loss_function = new MPCLossFunc(SP_PI_QUART,-SP_PI_QUART,200,-200);

  double parameters[6];
  for (int ii = 0; ii < 6; ++ii) {
    parameters[ii] = controls.data()[ii];
  }
  problem.AddResidualBlock(cost_function, NULL, parameters);
//  problem.AddResidualBlock(loss_function,NULL,parameters);

  std::vector<int> fix_param_vec;
  // fix the first two parameters
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);

  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(6,fix_param_vec);
  problem.SetParameterization(parameters,subparam);
  problem.SetParameterLowerBound(parameters,0,-((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterUpperBound(parameters,0,((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterLowerBound(parameters,1,-200);
  problem.SetParameterUpperBound(parameters,1,200);
  problem.SetParameterLowerBound(parameters,2,-((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterUpperBound(parameters,2,((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterLowerBound(parameters,3,-200);
  problem.SetParameterUpperBound(parameters,3,200);
  problem.SetParameterLowerBound(parameters,4,-((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterUpperBound(parameters,4,((SP_PI/4)-FINITE_DIFF_EPSILON));
  problem.SetParameterLowerBound(parameters,5,-200);
  problem.SetParameterUpperBound(parameters,5,200);

  // Run the solver!
  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
//  SoverIterationCallback iteration_callback(nullptr,nullptr);
//  options.callbacks.push_back(&iteration_callback);
  //  options.check_gradients = true;
  //  options.gradient_check_numeric_derivative_relative_step_size = 0.05;
  //  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
//  options.max_num_iterations = 10;
  options.min_relative_decrease = 1e-4;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int ii = 0; ii < 6; ++ii) {
    controls.data()[ii] = parameters[ii];
  }

//  std::cout << "MPCcontrols: \n" << controls << std::endl;

}
