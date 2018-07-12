#include <iostream>
#include <spirit/Gui/spopenscenegraphgui.h>
#include <spirit/Objects/spBox.h>
#include <eigen3/Eigen/Eigen>
#include <spirit/spirit.h>

int main(int argc, char** argv)
{
    // Testing OSG Gui
    // create box object
    Eigen::Vector3d boxsize;
    boxsize<< 50, 50, 1;

    Eigen::Vector3d boxcolor;
    boxColor<< 1, 0, 0;

    spPose p = spPose::Identity();
    p.translate(spTranslation(0,0,0));
    Eigen::AngleAxisd rot1(SP_PI/10, Eigen::Vector3d::UnitZ());
    p.rotate(rot1);


    Gui gui(spGuiType::GUI_OSG);
    Objects obj;

    spObjectHandle obj_box_index = obj.CreateBox(p, boxsize, 0, boxcolor);
    gui.AddObject(obj.GetObject(obj_box_index));



    while(!gui.ShouldQuit())
    {
        gui.Iterate(obj);
        //obj.StepPhySimulation(0.01);



    }

    return 0;
}
