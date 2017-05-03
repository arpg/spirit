#include <spirit/Planners/spLocalPlanner.h>

spLocalPlanner::spLocalPlanner(const spVehicleConstructionInfo& vehicle_info, Gui* gui):
  vehicle_parameters(vehicle_info) {
  gui_ = gui;
}

spLocalPlanner::~spLocalPlanner() {
}

void spLocalPlanner::SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint) {
  ceres::Problem problem;
  spState goal_state;
  goal_state.pose = end_waypoint.GetPose();
  goal_state.linvel = end_waypoint.GetLinearVelocity();
  Eigen::VectorXd residual_weight(13);
  residual_weight << 4, 4, 4, 3, 3, 3,0.07, 0.07, 0.07, 1, 1, 1,0.5;
  ceres::CostFunction* cost_function = new VehicleCeresCostFunc(vehicle_parameters,current_state,goal_state,residual_weight);
  double parameters[7];
  for (int ii = 0; ii < 6; ++ii) {
    parameters[ii] = controls.data()[ii];
  }
  parameters[6] = simulation_duration;
  problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), parameters);
  //    problem.AddResidualBlock(cost_function, NULL, parameters);
  std::vector<int> fix_param_vec;
  fix_param_vec.push_back(0);
  fix_param_vec.push_back(1);
//    fix_param_vec.push_back(2);
//    fix_param_vec.push_back(3);
//    fix_param_vec.push_back(4);
//    fix_param_vec.push_back(5);
//    fix_param_vec.push_back(6);

  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(7,fix_param_vec);
 //  problem.SetParameterization(inputcmd_curve.data(),subparam);
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
  problem.SetParameterLowerBound(parameters,6,0.6);
  problem.SetParameterUpperBound(parameters,6,20);
  // Run the solver!
  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
//  SoverIterationCallback iteration_callback(nullptr,nullptr);
//  options.callbacks.push_back(&iteration_callback);
  //  options.check_gradients = true;
  //  options.gradient_check_numeric_derivative_relative_step_size = 0.05;
  //  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
//    options.max_num_iterations = 30;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int ii = 0; ii < 6; ++ii) {
    controls.data()[ii] = parameters[ii];
  }
  simulation_duration = parameters[6];
}

void spLocalPlanner::SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint, /*spState& final_state,*/ std::shared_ptr<spStateSeries> traj_states) {
  SolveLocalPlan(controls,simulation_duration,current_state,end_waypoint);
  CarSimFunctor sim(vehicle_parameters,current_state);
  sim(0,(int)(simulation_duration/0.1),0.1,controls,0,-1,traj_states);
//  final_state = sim.GetState();
}

void spLocalPlanner::SolveLocalPlan(spTrajectory& trajectory) {
  for(int ii=0; ii<trajectory.GetNumWaypoints(); ii++) {
    SolveLocalPlan(trajectory,ii);
  }
}


void spLocalPlanner::SolveLocalPlan(spTrajectory& trajectory, int way_index, bool overwrite_endstate) {
    int next_index;
    if(way_index == trajectory.GetNumWaypoints()-1) {
      if(trajectory.IsLoop()) {
        next_index = 0;
      } else {
        return;
      }
    } else {
      next_index = way_index + 1;
    }
    spState current_state;
    current_state.pose = trajectory.GetWaypoint(way_index).GetPose();
    current_state.linvel = trajectory.GetWaypoint(way_index).GetLinearVelocity();
//    spState final_state;
    std::shared_ptr<spStateSeries> state_series = std::make_shared<spStateSeries>();
    double travel_duration = trajectory.GetTravelDuration(way_index);
    SolveLocalPlan(trajectory.GetControls(way_index),travel_duration,current_state, trajectory.GetWaypoint(next_index)/*,final_state*/,state_series);
    trajectory.SetTravelDuration(way_index,travel_duration);
    trajectory.SetTrajectoryStateSeries(way_index,state_series);
    if(overwrite_endstate && (next_index != 0)) {
      trajectory.GetWaypoint(next_index).SetPose(state_series->back()->pose);
      trajectory.GetWaypoint(next_index).SetLinearVelocityNorm(state_series->back()->linvel.norm());
    }
}

void spLocalPlanner::SolveInitialPlan(spTrajectory& trajectory, int way_index) {
    spState state;
    state.pose = trajectory.GetWaypoint(way_index).GetPose();
    state.linvel = trajectory.GetWaypoint(way_index).GetLinearVelocity();
    CarSimFunctor sim(vehicle_parameters,state/*,gui_*/);
    std::shared_ptr<spStateSeries> state_series = std::make_shared<spStateSeries>();
    double travel_duration = trajectory.GetTravelDuration(way_index);
    sim(0,(int)(travel_duration/0.1),0.1,trajectory.GetControls(way_index),0,-1,state_series);
    trajectory.SetTrajectoryStateSeries(way_index,state_series);
    sim.GetState();
}



