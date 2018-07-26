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
    Eigen::AngleAxisd rot(0, Eigen::Vector3d::UnitZ());
    gnd.rotate(rot);

    // NEED TO CHANGE OBJECTS BULLET CAN BE SET TO NONE
    Objects objs(spPhyEngineType::PHY_NONE);
    //Objects objs(spPhyEngineType::PHY_BULLET);
    spObjectHandle box_handle = objs.CreateBox(gnd, spBoxSize(10,10,1), 0, spColor(1,1,1));

    // create bike
    BikeParams params;
    spObjectHandle bike_handle = objs.CreateVehicle(params.bike_param);

    // mesh
    osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
    spObjectHandle mesh_handle = objs.CreateMesh(meshnode);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    gui.AddObject(objs.GetObject(box_handle));
    gui.AddObject(objs.GetObject(bike_handle));
    gui.AddObject(objs.GetObject(mesh_handle));
    spBike& bike = ((spBike&)objs.GetObject(bike_handle));


    spState state;
    state.pose = spPose::Identity();
    state.pose.translate(spTranslation(0,0,0));
    Eigen::AngleAxisd rot1(0,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot1);

    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

    spCtrlPts2ord_2dof inputcmd_curve;
    inputcmd_curve.col(0) = Eigen::Vector2d(0.3,1);
    inputcmd_curve.col(1) = Eigen::Vector2d(0.3,1);
    inputcmd_curve.col(2) = Eigen::Vector2d(0.3,1);

    while(!gui.ShouldQuit())
    {

        BikeSimFunctorRK4 mysim(params.bike_param,state);
        mysim(0,1,0.1,inputcmd_curve,0,0,nullptr,state_ptr);
        bike.SetState(mysim.GetState());
        gui.Iterate(objs);


        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        double lin_vel = std::sqrt(std::pow(mysim.GetState().linvel[0],2) + std::pow(mysim.GetState().linvel[1],2) + std::pow(mysim.GetState().linvel[2],2));
        std::cout<<"Yaw: "<<yaw<<" linear vel: "<<lin_vel<<std::endl;

    }
    return 0;
}

/*
        //spPose ps(mybox.GetPose());
        //ps.translate(spTranslation(0,0,h));
        //mybox.SetPose(ps);
        //h += -.1;
        //objs.StepPhySimulation(0.1);

*/

