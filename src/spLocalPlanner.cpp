#include <spirit/Planners/spLocalPlanner.h>

spLocalPlanner::spLocalPlanner(const spVehicleConstructionInfo& vehicle_info, bool overwrite_endstate, Gui* gui):
  vehicle_parameters(vehicle_info) {
//  weight_vec_ << 10, 10, 10, 0.1, 0.1, 0.1, 0.09, 0.09, 0.09, 0.1, 0.1, 0.1,0.1;
  weight_vec_ << 10, 10, 10, 5, 5, 5, 1, 1, 1, 5, 5, 5,0.1;
  gui_ = gui;
  overwrite_endstate_ = overwrite_endstate;
}

spLocalPlanner::~spLocalPlanner() {
}

void spLocalPlanner::SetCostWeight(const spBVPWeightVec& vec) {
  weight_vec_ = vec;
}

double spLocalPlanner::SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint) {
  ceres::Problem problem;
  spState goal_state;
  goal_state.pose = spPose(end_waypoint.GetPose());
  goal_state.linvel = spLinVel(end_waypoint.GetLinearVelocityInWorld());
  Eigen::VectorXd residual_weight(13);
//  residual_weight << 10, 10, 10, 0.1, 0.1, 0.1, 0.09, 0.09, 0.09, 0.1, 0.1, 0.1,0.1;
  ceres::CostFunction* cost_function = new VehicleCeresCostFunc(vehicle_parameters,current_state,goal_state,weight_vec_);
  Eigen::VectorXd min_limits(7);
  Eigen::VectorXd max_limits(7);
  // set steering limits
  for(int ii=0; ii<6; ii+=2) {
    min_limits[ii] = -SP_PI_QUART;
    max_limits[ii] = SP_PI_QUART;
  }
  // set engine limits
  for(int ii=1; ii<6; ii+=2) {
    min_limits[ii] = -20;
    max_limits[ii] = 20;
  }
  // set time of travel limits
  min_limits[6] = 0.2;
  max_limits[6] = 3;
  // create loss function
  ceres::CostFunction* loss_function = new ParamLimitLossFunc<7>(min_limits,max_limits,100);

  double parameters[7];
  for (int ii = 0; ii < 6; ++ii) {
    parameters[ii] = controls.data()[ii];
  }
  parameters[6] = simulation_duration;
  problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), parameters);
  problem.AddResidualBlock(loss_function,NULL,parameters);

  std::vector<int> fix_param_vec;
  // fix the first two parameters
//  fix_param_vec.push_back(0);
//  fix_param_vec.push_back(1);

//  ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(7,fix_param_vec);
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
//  problem.SetParameterLowerBound(parameters,6,0.6);
//  problem.SetParameterUpperBound(parameters,6,20);

  // Run the solver!
  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
//  SoverIterationCallback iteration_callback(nullptr,nullptr);
//  options.callbacks.push_back(&iteration_callback);
  //  options.check_gradients = true;
  //  options.gradient_check_numeric_derivative_relative_step_size = 0.05;
  //  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
//    options.max_num_iterations = 30;
  options.initial_trust_region_radius = 0.1;
  options.max_trust_region_radius = 0.1;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int ii = 0; ii < 6; ++ii) {
    controls.data()[ii] = parameters[ii];
  }
  simulation_duration = parameters[6];
  std::cout << "controls are \n" << controls << std::endl;
  return summary.final_cost;
}

double spLocalPlanner::SolveLocalPlan(spCtrlPts2ord_2dof& controls, double& simulation_duration, const spState& current_state, const spWaypoint& end_waypoint, std::shared_ptr<spStateSeries> traj_states) {
  double final_cost = SolveLocalPlan(controls,simulation_duration,current_state,end_waypoint);
  // TODO: we should be able to get traj_states from solution of last optimization step but for now we resimulate.
  //CarSimFunctor sim(vehicle_parameters,current_state,gui_);
  CarSimFunctor sim(vehicle_parameters,current_state,gui_);
//  CarSimFunctor sim(vehicle_parameters,current_state,nullptr);
  sim(0,(int)(simulation_duration/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,controls,0,-1,traj_states);
  return final_cost;
}

void spLocalPlanner::SolveLocalPlan(spTrajectory& trajectory) {
  for(int ii=0; ii<trajectory.GetNumWaypoints(); ii++) {
    SolveLocalPlan(trajectory,ii);
  }
}

double spLocalPlanner::SolveLocalPlan(spTrajectory& trajectory, int way_index) {
  int next_index;
  if(way_index == trajectory.GetNumWaypoints()-1) {
    if(trajectory.IsLoop()) {
      next_index = 0;
    } else {
      return -1;
    }
  } else {
    next_index = way_index + 1;
  }
  spState current_state;

  if(way_index == 0){
    // for now use pose and linvel of the waypoint, we need a more complicated waypoint in order to add other constraints
    current_state.pose = trajectory.GetWaypoint(way_index).GetPose();
    current_state.linvel = trajectory.GetWaypoint(way_index).GetLinearVelocityInWorld();
    current_state.rotvel = trajectory.GetWaypoint(way_index).GetRotVel();
    // create sub states for each wheel
    //    for(int ii = 0; ii<vehicle_parameters.wheels_anchor.size(); ii++) {
    //      current_state.InsertSubstate();
    //      current_state.substate_vec[ii]->linvel = current_state.linvel;
    //    }
  }else{
    if(overwrite_endstate_) {
      current_state = (*((*trajectory.GetTrajectoryStateSeries(way_index-1))[trajectory.GetTrajectoryStateSeries(way_index-1)->size()-1]));
      trajectory.GetControls(way_index).col(0) = trajectory.GetControls(way_index-1).col(2);
    } else {
      current_state.pose = trajectory.GetWaypoint(way_index).GetPose();
      current_state.linvel = trajectory.GetWaypoint(way_index).GetLinearVelocityInWorld();
      current_state.rotvel = trajectory.GetWaypoint(way_index).GetRotVel();
    }
  }
  std::shared_ptr<spStateSeries> state_series = std::make_shared<spStateSeries>();
  double travel_duration = 1;
  double final_cost = SolveLocalPlan(trajectory.GetControls(way_index),travel_duration,current_state, trajectory.GetWaypoint(next_index),state_series);
  trajectory.SetTravelDuration(way_index,(int)(travel_duration/0.1)*0.1);
  trajectory.SetTrajectoryStateSeries(way_index,state_series);
//  std::cout << "controls are :\n" << trajectory.GetControls(way_index) << std::endl;
//  std::cout << "travel duration is : " << trajectory.GetTravelDuration(way_index) << std::endl;
//  std::cout << "number of states is " << trajectory.GetTrajectoryStateSeries(way_index)->size() << std::endl;
  // adjust the next waypoint accordingly if overwrite_endstate was enabled
  if(overwrite_endstate_ && (next_index != 0)) {
    trajectory.GetWaypoint(next_index).SetPose(state_series->back()->pose);
    trajectory.GetWaypoint(next_index).SetLinearVelocityNorm(state_series->back()->linvel.norm());
  }

  return final_cost;
}

void spLocalPlanner::SolveInitialPlan(spTrajectory& trajectory, int way_index) {
  if(trajectory.GetTravelDuration(way_index) == -1) {
    SPERROREXIT("Travel duration between waypoints hasn't been initialized for waypoint " + std::to_string(way_index));
  }
  spState state;
  // for now use pose and linvel of the waypoint, we need a more complicated waypoint in order to add other constraints
  state.pose = trajectory.GetWaypoint(way_index).GetPose();
  state.linvel = trajectory.GetWaypoint(way_index).GetLinearVelocityInWorld();
  state.rotvel = trajectory.GetWaypoint(way_index).GetRotVel();
  // TODO: fix this zero index issue
  if((way_index > 0)&&(overwrite_endstate_)) {
    // TODO: find simpler struct for state series
    state = (*((*trajectory.GetTrajectoryStateSeries(way_index-1))[trajectory.GetTrajectoryStateSeries(way_index-1)->size()-1]));
    trajectory.GetControls(way_index).col(0) = trajectory.GetControls(way_index-1).col(2);
  }
  // create sub states for each wheel
//    for(int ii = 0; ii<vehicle_parameters.wheels_anchor.size(); ii++) {
//      state.InsertSubstate();
//      state.substate_vec[ii]->linvel = state.linvel;
//    }
  CarSimFunctor sim(vehicle_parameters,state,gui_);
  std::shared_ptr<spStateSeries> state_series = std::make_shared<spStateSeries>();
  double travel_duration = trajectory.GetTravelDuration(way_index);
  sim(0,(int)(travel_duration/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,trajectory.GetControls(way_index),0,-1,state_series);

  trajectory.SetTrajectoryStateSeries(way_index,state_series);
}



