#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/BikeParameters.h>
#include <math.h>

int main(int argc, char** argv){

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
    //osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
    //spObjectHandle mesh_handle = objs->CreateMesh(meshnode);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    gui.AddObject(objs->GetObject(box_handle));
    gui.AddObject(objs->GetObject(bike_handle));
    //gui.AddObject(objs->GetObject(mesh_handle));
    spBike& bike = ((spBike&)objs->GetObject(bike_handle));

    /*
    spPoints3d p;
    spPoints3d p2;
    spObjectHandle linestrip_handle = objs->CreateLineStrip(spPose::Identity(),p,spColor(0, 1, 0));
    spObjectHandle linestrip_handle2 = objs->CreateLineStrip(spPose::Identity(),p2,spColor(1, 0, 0));
    gui.AddObject(objs->GetObject(linestrip_handle));
    gui.AddObject(objs->GetObject(linestrip_handle2));

    double x=0;
    double y=0;
    double z=0,zp=0;
    int r = 5;
    double t=0, dt=.7;
    for(int ii=0;ii<200;ii++){
        x = r*std::cos(t);
        y = r*std::sin(t);
        z += dt;
        t += dt;
        zp -= dt;
        p.push_back(spPoint3d(x,y,z));
        p2.push_back(spPoint3d(x,y,zp));
    }

    ((spLineStrip&)objs->GetObject(linestrip_handle)).SetLineStripPoints(p);
    ((spLineStrip&)objs->GetObject(linestrip_handle2)).SetLineStripPoints(p2);

    while(!gui.ShouldQuit()){
        gui.Iterate(objs);
        spGeneralTools::Delay_ms(100);

    } //*/



    // /*
    // BVP
    spTrajectory traj(gui, objs);

    // waypoints
    /*
    spPose pose0(spPose::Identity());
    pose0.translate(spTranslation(2,-3,0.06));
    Eigen::AngleAxisd rot0(0,Eigen::Vector3d::UnitZ());
    pose0.rotate(rot0);
    traj.AddWaypoint(pose0,1,spLinVel(1,0,0));

    spPose pose1(spPose::Identity());
    pose1.translate(spTranslation(2,3,0));
    double angle = SP_PI;
    Eigen::AngleAxisd rot1(angle,Eigen::Vector3d::UnitZ());
    pose1.rotate(rot1);
    traj.AddWaypoint(pose1,1,spLinVel(1,0,0));

    spPose pose2(spPose::Identity());
    pose2.translate(spTranslation(-2,2,0.06));
    angle = -SP_PI/2;
    Eigen::AngleAxisd rot2(angle,Eigen::Vector3d::UnitZ());
    pose2.rotate(rot2);
    traj.AddWaypoint(pose2,1,spLinVel(1,0,0));

    spPose pose3(spPose::Identity());
    pose3.translate(spTranslation(-2,-2,0.06));
    angle = 0;
    Eigen::AngleAxisd rot3(0,Eigen::Vector3d::UnitZ());
    pose3.rotate(rot3);
    traj.AddWaypoint(pose3,1,spLinVel(1,0,0));
    */
    double a = 1.5;
    double b = 1.5;
    int num_waypoints = 8;
    for(int ii=0; ii<num_waypoints; ii++) {
      // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
      double theta = ii*(2*SP_PI)/num_waypoints;
      double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
      double x = r*cos(theta);
      double y = r*sin(theta);
      // slope of the line is
      double angle = atan2(-(x*b*b),(y*a*a));
      spPose pose(spPose::Identity());
      pose.translate(spTranslation(x,y,0.07));
      Eigen::AngleAxisd rot(angle+SP_PI_HALF/*+0.6*/,Eigen::Vector3d::UnitZ());
      pose.rotate(rot);
      traj.AddWaypoint(pose,1,spLinVel(1,0,0));
      spRotVel rotvel(0,0,2);
      traj.GetWaypoint(ii).SetRotVel(rotvel);
      traj.GetWaypoint(ii).SetLinearVelocityDirection(spLinVel(0,1,0));
  }

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

    // control signal from local planner
    /*
    BikeSimFunctorRK4 mysim(params.bike_param,state);
    while(!gui.ShouldQuit()){
        spTimestamp t0 = spGeneralTools::Tick();

        mysim(0,1,0.1,traj.GetControls(0),0,-1,0,state_ptr); // gets control signal between first two waypoints
        bike.SetState(mysim.GetState());
        gui.Iterate(objs);

        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        //double lin_vel = mysim.GetState().linvel.norm();
        //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

        double calc_time = spGeneralTools::Tock_ms(t0);
        std::cout << "Frequency " << (double)(1/(calc_time/1000)) << " Hz" << std::endl;
    } // */

    // /*
    // MPC reference tracking
    float horizon = 1;
    spMPC<BikeSimFunctorRK4> mpc(params.bike_param, horizon);

    spCtrlPts2ord_2dof controls;
    controls.col(0) = Eigen::Vector2d(0,0);
    controls.col(1) = Eigen::Vector2d(0,0);
    controls.col(2) = Eigen::Vector2d(0,0);

    BikeSimFunctorRK4 mysim(params.bike_param, state);
    // /*
      while(!gui.ShouldQuit()){
          spTimestamp t0 = spGeneralTools::Tick();

          mpc.CalculateControls(traj, state, controls);
          std::cout << "controls -> \n" << controls << std::endl;

          mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,controls,0,-1,0,state_ptr);
          for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){
              bike.SetState(mysim.GetState());
              gui.Iterate(objs);

              //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
              //double lin_vel = mysim.GetState().linvel.norm();
              //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

              double calc_time = spGeneralTools::Tock_ms(t0);
              //std::cout << "Frequency " << (double)(1/(calc_time/1000)) << " Hz" << std::endl;

              std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
       }

     // */

    //spCurve controls_curve(2,2);
    //traj.SetTravelDuration(0,1);
    //localplanner.SolveInitialPlan(traj,0);
    //double final_cost = localplanner.SolveLocalPlan(traj,0);
    //std::cout << "final cost: " << final_cost << std::endl;
    //controls_curve.SetBezierControlPoints(traj.GetControls(0));
    //std::cout << "bezier control points:\n"<<controls_curve.GetBezierControlPoints() << std::endl;

     /*
    // MPC controller circle maneuver
    float horizon = 5;
    double radius = 1;  // radius of circle
    double vel = 1;
    spMPC<BikeSimFunctorRK4> mpc(params.bike_param,horizon);

    spCtrlPts2ord_2dof inputcmd_curve;
    inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(2) = Eigen::Vector2d(0,0);

    BikeSimFunctorRK4 mysim(params.bike_param,state);

    if(mpc.CircleManReg(state,inputcmd_curve,radius,vel)) {
      std::cout << "controls -> \n" << inputcmd_curve << std::endl;

      std::shared_ptr<spStateSeries> series = std::make_shared<spStateSeries>();
      mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,inputcmd_curve,0,-1,series,state_ptr);
      while(1){
      for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){
        spState state(*((*series)[ii]));
        bike.SetState(state);
        gui.Iterate(objs);
        double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        std::cout<<"Yaw ->"<<yaw<<" "<<"speed -> " << state.linvel.norm() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "done " << std::endl;
        gui.Iterate(objs);
      }
    } // */

    return 0;
}


