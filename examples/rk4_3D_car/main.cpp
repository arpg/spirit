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
    osg::ref_ptr<osg::Node> meshnode = osgDB::readNodeFile( "lab_v2.ply" );
    spObjectHandle mesh_handle = objs->CreateMesh(meshnode);

    spMeshVisitor mv;
    meshnode->accept(mv);

    // boundary
    Eigen::VectorXd b(5);
    b<< -4, 2, -4, 1, 1; //xmin, xmax, ymin, ymax, zlim
    mv.GetMeshData();
    mv.BoundingBox(b);

    // set gui and add objects
    Gui gui;
    gui.Create(spGuiType::GUI_OSG);
    //gui.AddObject(objs->GetObject(box_handle));
    gui.AddObject(objs->GetObject(bike_handle));
    gui.AddObject(objs->GetObject(mesh_handle));
    spBike& bike = ((spBike&)objs->GetObject(bike_handle));

    // set cars initial pose
    spState state;
    Eigen::AngleAxisd rot(SP_PI,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot);
    state.pose.translate(spTranslation(0,0,0));
    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

    // /*
    spCtrlPts2ord_2dof inputcmd_curve;
    double sf = 0.05;
    inputcmd_curve.col(0) = Eigen::Vector2d(sf,1);
    inputcmd_curve.col(1) = Eigen::Vector2d(sf,1);
    inputcmd_curve.col(2) = Eigen::Vector2d(sf,1);

    MeshBikeSimFunctorRK4 mysim(params.bike_param,state);
    mysim.SetMeshData(mv.mstruct.bvtx_ptr, mv.mstruct.bnrml_ptr);

    while(!gui.ShouldQuit()){
        spTimestamp t0 = spGeneralTools::Tick();

        mysim(0,1,0.1,inputcmd_curve,0,0,nullptr,state_ptr);
        bike.SetState(mysim.GetState());

        gui.Iterate(objs);

        //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
        //double lin_vel = mysim.GetState().linvel.norm();
        //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

        double calc_time = spGeneralTools::Tock_ms(t0);
        //std::cout << "Frequency " << (double)(1/(calc_time/1000)) << " Hz" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } // */


    return 0;
}


