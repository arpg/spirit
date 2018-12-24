#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/BikeParameters.h>
#include <math.h>
#include <spirit/spMeshFunctions.h>
#include <vector>
//gprof tool
//openmp

int main(int argc, char** argv){

    // Testing threads
    //unsigned int n = std::thread::hardware_concurrency();
    //std::cout<<n<<" concurrent threads supported"<<std::endl;

    // Testing OSG Gui
    // create ground
    spPose gnd = spPose::Identity();
    gnd.translate(spTranslation(0,0,-0.525));

    std::shared_ptr<Objects> objs = std::make_shared<Objects>(spPhyEngineType::PHY_NONE);
    //std::shared_ptr<Objects> objs = std::make_shared<Objects>(spPhyEngineType::PHY_BULLET);
    spObjectHandle box_handle = objs->CreateBox(gnd, spBoxSize(20,20,1), 0, spColor(1,1,1));

    // create bike
    BikeParams params;
    spObjectHandle bike_handle = objs->CreateVehicle(params.bike_param);

    // mesh
    osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
    spObjectHandle mesh_handle = objs->CreateMesh(meshnode);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    //gui.AddObject(objs->GetObject(box_handle));
    gui.AddObject(objs->GetObject(bike_handle));
    gui.AddObject(objs->GetObject(mesh_handle));
    spBike& bike = ((spBike&)objs->GetObject(bike_handle));

 /*
    spTrajectory traj(gui, objs);

    // waypoints
    spPose pose0(spPose::Identity());
    pose0.translate(spTranslation(2,-1,0.06));
    Eigen::AngleAxisd rot0(-SP_PI,Eigen::Vector3d::UnitZ());
    pose0.rotate(rot0);
    traj.AddWaypoint(pose0,1,spLinVel(1,0,0));

    spPose pose1(spPose::Identity());
    pose1.translate(spTranslation(-4.3,-1,1));
    Eigen::AngleAxisd rot1(SP_PI,Eigen::Vector3d::UnitZ());
    pose1.rotate(rot1);
    traj.AddWaypoint(pose1,1,spLinVel(1,0,0));


    spPose pose2(spPose::Identity());
    pose2.translate(spTranslation(0,0.5,0.06));
    Eigen::AngleAxisd rot2(0,Eigen::Vector3d::UnitZ());
    pose2.rotate(rot2);
    traj.AddWaypoint(pose2,1,spLinVel(1,0,0));

    spPose pose3(spPose::Identity());
    pose3.translate(spTranslation(1,0,0.06));
    Eigen::AngleAxisd rot3(-SP_PI/2,Eigen::Vector3d::UnitZ());
    pose3.rotate(rot3);
    traj.AddWaypoint(pose3,1,spLinVel(1,0,0));


    // is trajectory in a loop
    traj.IsLoop(true);

    // Solve local plan
    // set to true, each waypoint in connected to each other in order created
    spLocalPlanner<MeshBikeSimFunctorRK4> localplanner(params.bike_param, true, &gui);
    spBVPWeightVec weight_vec;
    // x,y,z,roll,pitch,yaw,xdot,ydot,zdot,roll_dot,pitch_dot,yaw_dot,time
    weight_vec << 100, 100, 0, 0, 0, 10, 0.009, 0.009, 0.009, 0.01, 0.01, 0.01, 0.1;
    localplanner.SetCostWeight(weight_vec);

    for(int ii=0; ii<traj.GetNumWaypoints(); ii++){
        spTimestamp t0 = spGeneralTools::Tick();
        traj.SetTravelDuration(ii, 0.5); // 0.5 init cond
        localplanner.SolveInitialPlan(traj, ii); // seed init cond for path
        localplanner.SolveLocalPlan(traj); // solve path
        //std::cout << "Frequency " << (double)(1/(spGeneralTools::Tock_ms(t0)/1000)) << " Hz" << std::endl;

    }
 */

    // set cars initial pose
    spState state;
    //state.pose = traj.GetWaypoint(0).GetPose();
    Eigen::AngleAxisd rot(SP_PI,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot);
    state.pose.translate(spTranslation(0,0,0));
    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);


    // MPC reference tracking
    float horizon = .01;
    spMPC<MeshBikeSimFunctorRK4> mpc(params.bike_param, horizon);
    MeshBikeSimFunctorRK4 mysim(params.bike_param,state);


    spCtrlPts2ord_2dof inputcmd_curve;
    double sf = 0;
    double a = 1;
    inputcmd_curve.col(0) = Eigen::Vector2d(sf,a);
    inputcmd_curve.col(1) = Eigen::Vector2d(sf,a);
    inputcmd_curve.col(2) = Eigen::Vector2d(sf,a);

    while(!gui.ShouldQuit()){
        spTimestamp t0 = spGeneralTools::Tick();

         ///*
        mysim(0,1,0.1,inputcmd_curve,0,0,nullptr,state_ptr);
        bike.SetState(mysim.GetState());
        gui.Iterate(objs);
        // */

          /*
        std::cout<<"Sim starting"<<std::endl;
        mpc.CalculateControls(traj, state, inputcmd_curve);
        mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,inputcmd_curve,0,-1,0,state_ptr);
        for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){

            bike.SetState(mysim.GetState());
            gui.Iterate(objs);
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } // */


        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        //double lin_vel = mysim.GetState().linvel.norm();
        //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;
        double calc_time = spGeneralTools::Tock_ms(t0);
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << "Frequency " << (double)(1/(calc_time/1000)) << " Hz" << std::endl;
    }
    return 0;
}


