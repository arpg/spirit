#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/BikeParameters.h>
#include <math.h>
#include <thread>
#include <random>

int main(int argc, char** argv){

    // Testing threads
    //unsigned int n = std::thread::hardware_concurrency();
    //std::cout<<n<<" concurrent threads supported"<<std::endl;

    // create ground
    spPose gnd = spPose::Identity();
    gnd.translate(spTranslation(0,0,-0.525));

    std::shared_ptr<Objects> objs = std::make_shared<Objects>(spPhyEngineType::PHY_NONE);
    spObjectHandle gnd_handle = objs->CreateBox(gnd, spBoxSize(20,20,1), 0, spColor(1,1,1));

    // create bike
    BikeParams params;
    spObjectHandle bike_handle = objs->CreateVehicle(params.bike_param);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    gui.AddObject(objs->GetObject(gnd_handle));
    gui.AddObject(objs->GetObject(bike_handle));
    spBike& bike = ((spBike&)objs->GetObject(bike_handle));

    ///*
    spTrajectory traj(gui, objs);

    // waypoints
    spPose pose0(spPose::Identity());
    pose0.translate(spTranslation(0,-2,0.06));
    Eigen::AngleAxisd rot0(0,Eigen::Vector3d::UnitZ());
    pose0.rotate(rot0);
    traj.AddWaypoint(pose0,1,spLinVel(1,0,0));

    spPose pose1(spPose::Identity());
    pose1.translate(spTranslation(2,0,0.06));
    double angle = SP_PI/2;
    Eigen::AngleAxisd rot1(angle,Eigen::Vector3d::UnitZ());
    pose1.rotate(rot1);
    traj.AddWaypoint(pose1,1,spLinVel(1,0,0));

    spPose pose3(spPose::Identity());
    pose3.translate(spTranslation(0,2,0.06));
    angle = SP_PI;
    Eigen::AngleAxisd rot3(angle,Eigen::Vector3d::UnitZ());
    pose3.rotate(rot3);
    traj.AddWaypoint(pose3,1,spLinVel(1,0,0));

    spPose pose4(spPose::Identity());
    pose4.translate(spTranslation(-2,0,0.06));
    angle = -SP_PI/2;
    Eigen::AngleAxisd rot4(angle,Eigen::Vector3d::UnitZ());
    pose4.rotate(rot4);
    traj.AddWaypoint(pose4,1,spLinVel(1,0,0));

    traj.IsLoop(true);

    // Solve local plan
    // set to true, each waypoint in connected to each other in order created
    spLocalPlanner<BikeSimFunctorRK4> localplanner(params.bike_param, true, &gui);
    spBVPWeightVec weight_vec;
    // x,y,z,roll,pitch,yaw,xdot,ydot,zdot,roll_dot,pitch_dot,yaw_dot,time
    weight_vec << 100, 100, 0, 0, 0, 10, 0.009, 0.009, 0.009, 0.01, 0.01, 0.01, 0.1;
    localplanner.SetCostWeight(weight_vec);

    for(int ii=0; ii<traj.GetNumWaypoints(); ii++){
        traj.SetTravelDuration(ii, 0.5); // 0.5 init cond
        localplanner.SolveInitialPlan(traj, ii); // seed init cond for path
        localplanner.SolveLocalPlan(traj); // solve path
    }

    // set cars initial pose
    spState state;
    state.pose = traj.GetWaypoint(0).GetPose();
    state.pose.translate(spTranslation(0,0,0));
    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);
    // */

    // set cars initial pose
    /*
    spState state;
    Eigen::AngleAxisd rot(0,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot);
    state.pose.translate(spTranslation(0,0,0));
    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);
    */


    // MPC reference tracking
    float horizon = .1;
    spMPC<BikeSimFunctorRK4> mpc(params.bike_param, horizon);
    BikeSimFunctorRK4 mysim(params.bike_param,state);

    spCtrlPts2ord_2dof controls;
    controls.col(0) = Eigen::Vector2d(0,0);
    controls.col(1) = Eigen::Vector2d(0,0);
    controls.col(2) = Eigen::Vector2d(0,0);

    // Generate Random Pertubations in init Controls
    /*
    int num_traj = 8;
    double accel_mean = 0;
    double accel_sigma = 0.3;
    double steering_mean = 0;
    double steering_sigma = 1;
    Eigen::MatrixXd rand_cntrl(8,2);
    std::default_random_engine generator;
    std::normal_distribution<double> accel_dist(accel_mean,accel_sigma);
    std::normal_distribution<double> steering_dist(steering_mean,steering_sigma);
    for(int ii=0; ii<num_traj; ii++){
        rand_cntrl(ii,0) = steering_dist(generator);
        rand_cntrl(ii,1) = accel_dist(generator);
        //std::cout<<"s: "<<rand_cntrl(ii,1)<<std::endl;
        //std::cout<<"a: "<<rand_cntrl(ii,0)<<std::endl;

    }
    */
    double freq, calc_time, simtime, s, a;
    simtime = 2;

    while(!gui.ShouldQuit()){
        spTimestamp t0 = spGeneralTools::Tick();

         /*
        for(int ii=0; ii<num_traj; ii++){
            s = rand_cntrl(ii,0);
            a = rand_cntrl(ii,1);
            controls.col(0) = Eigen::Vector2d(s,a);
            controls.col(1) = Eigen::Vector2d(s,a);
            controls.col(2) = Eigen::Vector2d(s,a);

            mysim(0,simtime/0.1,0.1,controls,0,0,nullptr,state_ptr);
            bike.SetState(mysim.GetState());
            gui.Iterate(objs);

            calc_time = spGeneralTools::Tock_ms(t0);
            freq = (double)(1/(calc_time/1000));
            //std::cout << "Frequency " << freq << " Hz" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            Eigen::AngleAxisd rot(0,Eigen::Vector3d::UnitZ());
            state.pose.rotate(rot);
            state.pose.translate(spTranslation(0,0,0));
            bike.SetState(state);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        }
        */

        // /*
        mpc.CalculateControls(traj, state, controls);
        mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,controls,0,-1,0,state_ptr);
        for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){
            bike.SetState(mysim.GetState());
            gui.Iterate(objs);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
            //double lin_vel = mysim.GetState().linvel.norm();
            //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

            calc_time = spGeneralTools::Tock_ms(t0);
            freq = (double)(1/(calc_time/1000));
            std::cout << "Frequency " << freq << " Hz" << std::endl;

        }
        // */
    }
    return 0;
}








