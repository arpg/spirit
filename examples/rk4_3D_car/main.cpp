#include <iostream>
#include <spirit/Gui/spOpenSceneGraphGui.h>
#include <spirit/Objects/spBox.h>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>
#include <spirit/BikeParameters.h>

int main(int argc, char** argv)
{
    // Testing OSG Gui
    // create ground
    spPose gnd = spPose::Identity();
    gnd.translate(spTranslation(0,0,-0.5));
    Eigen::AngleAxisd rot1(0/*SP_PI/10*/, Eigen::Vector3d::UnitZ());
    gnd.rotate(rot1);

    Objects objs;
    spObjectHandle obj_box_index1 = objs.CreateBox(gnd, spBoxSize(10,10,0.2), 0, spColor(1,0,0));

    // create bike
    BikeParams params;
    spObjectHandle car_handle = objs.CreateVehicle(params.car_param);

    // waypoint
    spPose p = spPose::Identity();
    p.translate(spTranslation(1,1,0));
    spObjectHandle waypt_handle = objs.CreateWaypoint(p, spColor(0,1,0));

    //spBox& mybox = ((spBox&)objs.GetObject(obj_box_index2));

    Gui gui;
    gui.Create(spGuiType::GUI_OSG);

    //gui.AddObject(objs.GetObject(obj_box_index1));
    //gui.AddObject(objs.GetObject(car_handle));
    gui.AddObject(objs.GetObject(waypt_handle));


    while(!gui.ShouldQuit())
    {

        gui.Iterate(objs);
        //spPose ps(mybox.GetPose());
        //ps.translate(spTranslation(0,0,h));
        //mybox.SetPose(ps);
        //h += -.1;
        //objs.StepPhySimulation(0.1);
        spGeneralTools::Delay_ms(100);

    }

    return 0;
}
