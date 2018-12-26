#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/BikeParameters.h>
#include <math.h>

int main(int argc, char** argv){


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

    // Init vehicle
    spState state;
    double angle = SP_PI/2;
    Eigen::AngleAxisd rot1(angle,Eigen::Vector3d::UnitZ());
    state.pose.rotate(rot1);
    state.pose.translate(spTranslation(0,0,0));
    bike.SetState(state);
    std::shared_ptr<spState> state_ptr = std::make_shared<spState>(state);

    // MPC controller circle maneuver
    float horizon = .5;
    double radius = .5;  // radius of circle
    double vel = .1;
    spMPC<BikeSimFunctorRK4> mpc(params.bike_param,horizon);
    BikeSimFunctorRK4 mysim(params.bike_param, state);

    spCtrlPts2ord_2dof controls;
    controls.col(0) = Eigen::Vector2d(0,0);
    controls.col(1) = Eigen::Vector2d(0,0);
    controls.col(2) = Eigen::Vector2d(0,0);

    double freq, calc_time;

    if(mpc.CircleManReg(mysim.GetState(),controls,radius,vel)){
        std::cout << "controls -> \n" << controls << std::endl;

        while(!gui.ShouldQuit()){
            spTimestamp t0 = spGeneralTools::Tick();

            mysim(0,(int)(horizon/DISCRETIZATION_STEP_SIZE),DISCRETIZATION_STEP_SIZE,controls,0,-1,0,state_ptr);
            for(int ii=0; ii<(int)(horizon/DISCRETIZATION_STEP_SIZE); ii++){
                bike.SetState(mysim.GetState());
                gui.Iterate(objs);

                //double yaw = mysim.GetState().pose.rotation().eulerAngles(0,1,2)[2];
                //double lin_vel = mysim.GetState().linvel.norm();
                //std::cout<<"Yaw: "<<yaw<<" linear velocity: "<<lin_vel<<std::endl;

                calc_time = spGeneralTools::Tock_ms(t0);
                freq = (double)(1/(calc_time/1000));
                std::cout << "Frequency " << freq << " Hz" << std::endl;

                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    }
    return 0;
}
