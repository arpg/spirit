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
    spObjectHandle box_handle = objs->CreateBox(gnd, spBoxSize(10,10,1), 0, spColor(1,1,1));

    // create bike
    BikeParams params;
    spObjectHandle bike_handle = objs->CreateVehicle(params.bike_param);

    // mesh
    osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
    spObjectHandle mesh_handle = objs->CreateMesh(meshnode);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    gui.AddObject(objs->GetObject(box_handle));
    gui.AddObject(objs->GetObject(bike_handle));
    //gui.AddObject(objs->GetObject(mesh_handle));
    spBike& bike = ((spBike&)objs->GetObject(bike_handle));


    spState state;
    state.pose = spPose::Identity();
    state.pose.translate(spTranslation(0,0,0));
    Eigen::AngleAxisd rot1(0,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot1);

    //bike.SetState(state);
    //std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

    // MPC controller
    float horizon = 10;
    double radius = 1;
    double vel = 2;
    spMPC mpc(params.bike_param,horizon);

    spCtrlPts2ord_2dof inputcmd_curve;
    inputcmd_curve.col(0) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(1) = Eigen::Vector2d(0,0);
    inputcmd_curve.col(2) = Eigen::Vector2d(0,0);

    BikeSimFunctorRK4 mysim(params.bike_param,state);

    if(mpc.CircleManReg(state,inputcmd_curve,radius,vel)) {
      std::cout << "controls -> \n" << inputcmd_curve << std::endl;

      std::shared_ptr<spStateSeries> series = std::make_shared<spStateSeries>();
      mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,inputcmd_curve,0,-1,series,std::make_shared<spState>(state));
      while(1){
      for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){
        spState state(*((*series)[ii]));
        bike.SetState(state);
        gui.Iterate(objs);
        std::cout << "speed -> " << state.linvel.norm() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "done " << std::endl;
        gui.Iterate(objs);
      }
    }


    /*
    while(!gui.ShouldQuit())
    {

        BikeSimFunctorRK4 mysim(params.bike_param,state);
        mysim(0,1,0.1,inputcmd_curve,0,0,nullptr,state_ptr);
        bike.SetState(mysim.GetState());
        gui.Iterate(objs);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        double lin_vel = std::sqrt(std::pow(mysim.GetState().linvel[0],2) + std::pow(mysim.GetState().linvel[1],2) + std::pow(mysim.GetState().linvel[2],2));
        std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

    } // */
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return 0;
}


