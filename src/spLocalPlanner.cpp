#include <spirit/Planners/spLocalPlanner.h>

spLocalPlanner::spLocalPlanner(spTrajectory& initial_trajectory, const spVehicleConstructionInfo& vehicle_info):
  trajectory(initial_trajectory), vehicle_information(vehicle_info) {
}

spLocalPlanner::~spLocalPlanner() {
}

spState spLocalPlanner::SolveLocalPlan(int ii) {
//  for(int ii=0; ii<trajectory.GetNumWaypoints()-1; ii++) {
    ceres::Problem problem;
    spState curr_state;
    spState goal_state;
    curr_state.pose = trajectory.GetWaypoint(ii).GetPose();
    goal_state.pose = trajectory.GetWaypoint(ii+1).GetPose();
    goal_state.linvel = trajectory.GetWaypoint(ii+1).GetLinearVelocity();
//    goal_state.rotvel.angle() = 0;
    ceres::CostFunction* cost_function = new VehicleCeresCostFunc(vehicle_information,curr_state,goal_state);
    double parameters[7];
    for (int ii = 0; ii < 6; ++ii) {
      parameters[ii] = trajectory.GetControls(ii).data()[ii];
    }
    double sim_length = 4;
    parameters[6] =  sim_length;
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), parameters);
//    problem.AddResidualBlock(cost_function, NULL, parameters);
    std::vector<int> fix_param_vec;
    fix_param_vec.push_back(0);
    fix_param_vec.push_back(1);
//     fix_param_vec.push_back(2);
//    fix_param_vec.push_back(3);
//    fix_param_vec.push_back(4);
   //  fix_param_vec.push_back(5);
//    fix_param_vec.push_back(6);

    ceres::SubsetParameterization* subparam = new ceres::SubsetParameterization(7,fix_param_vec);
   //  problem.SetParameterization(inputcmd_curve.data(),subparam);
    problem.SetParameterization(parameters,subparam);
    problem.SetParameterLowerBound(parameters,0,-((SP_PI/4)-0.06));
    problem.SetParameterUpperBound(parameters,0,((SP_PI/4)-0.06));
    problem.SetParameterLowerBound(parameters,1,-200);
    problem.SetParameterUpperBound(parameters,1,200);
    problem.SetParameterLowerBound(parameters,2,-((SP_PI/4)-0.06));
    problem.SetParameterUpperBound(parameters,2,((SP_PI/4)-0.06));
    problem.SetParameterLowerBound(parameters,3,-200);
    problem.SetParameterUpperBound(parameters,3,200);
    problem.SetParameterLowerBound(parameters,4,-((SP_PI/4)-0.06));
    problem.SetParameterUpperBound(parameters,4,((SP_PI/4)-0.06));
    problem.SetParameterLowerBound(parameters,5,-200);
    problem.SetParameterUpperBound(parameters,5,200);
    problem.SetParameterLowerBound(parameters,6,0.6);
    problem.SetParameterUpperBound(parameters,6,20);
    // Run the solver!
    ceres::Solver::Options options;
   //  options.check_gradients = true;
   //  options.gradient_check_numeric_derivative_relative_step_size = 0.05;
    options.update_state_every_iteration = true;
   //  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    spCtrlPts2ord_2dof inputcmd_curve;
    inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(2) = Eigen::Vector2d(0,0);


//    std::cout << "params : " << std::endl;
    for (int ii = 0; ii < 6; ++ii) {
      inputcmd_curve.data()[ii] = parameters[ii];
      trajectory.GetControls(ii).data()[ii] = parameters[ii];
//      std::cout << trajectory.GetControls(ii).data()[ii] << ", ";
    }
//    std::cout << std::endl;
    sim_length = parameters[6];
    std::cout << "simlength is " << parameters[6] << std::endl;
    std::cout << "controls are \n" << inputcmd_curve << std::endl;
    trajectory.SetControls(ii,inputcmd_curve);
    CarSimFunctor sim(vehicle_information,curr_state);
//    sim(0,20,0.1,trajectory.GetControls(ii),0,-1);
    sim(0,(int)(sim_length/0.1),0.1,inputcmd_curve,0,-1);
    trajectory.SetTrajectoryPoints(ii,sim.GetTrajectoryPoints());
    return sim.GetState();
//  }
}

void spLocalPlanner::CalcInitialPlans() {
  for(int ii=0; ii<trajectory.GetNumWaypoints(); ii++) {
    spState state;
    state.pose = trajectory.GetWaypoint(ii).GetPose();
    CarSimFunctor sim(vehicle_information,state);
    sim(0,10,0.1,trajectory.GetControls(ii),0,-1);
    trajectory.SetTrajectoryPoints(ii,sim.GetTrajectoryPoints());
    sim.GetState();
  }
}

