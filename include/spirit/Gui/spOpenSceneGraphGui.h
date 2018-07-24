#ifndef SPOPENSCENEGRAPHGUI_H
#define SPOPENSCENEGRAPHGUI_H

#include <spirit/spSettings.h>
#include <spirit/Gui/spCommonGui.h>
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>
#include <string>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/Node>
#include <osg/ref_ptr>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>



//// this class is the interface between OpenSceneGraph and spGui
class spOpenSceneGraphGui : public spCommonGUI {
    public:
        spOpenSceneGraphGui();
        ~spOpenSceneGraphGui();

        void InitGui();
        bool ShouldQuit();
        void Refresh();
        void CheckKeyboardAction();
        void AddBox(spBox& box);
        void AddWaypoint(spWaypoint& waypoint);
        void AddVehicle(spVehicle& vehicle);
        void AddLineStrip(spLineStrip& linestrip);
        void AddMesh(spMesh& mesh);
        void UpdateGuiObjectsFromSpirit(Objects &spobj);
        void UpdateSpiritObjectsFromGui(Objects& spobjects);
        void RemoveVehicle(spVehicle& vehicle);

    private:
        //static void KeyActionMethodSample();
        void UpdateBoxGuiObject(spBox& spobj);
        //void UpdateWaypointGuiObject(spWaypoint& spobj);
        void UpdateVehicleGuiObject(spVehicle& spobj);
        //void UpdateLineStripGuiObject(spLineStrip& spobj);


        // variables relevent to OSG visualizer
        osgViewer::Viewer viewer_;
        osg::ref_ptr<osg::Group> root_ = new osg::Group();
        double simtime_;

        struct OSGobj
        {
            std::string name_;
            osg::ref_ptr<osg::MatrixTransform> transform_ = new osg::MatrixTransform;
            osg::Matrix translate_, rotx_, roty_, rotz_;

            osg::ref_ptr<osg::ShapeDrawable> shape_ = new osg::ShapeDrawable;
            osg::ref_ptr<osg::Geode> geoshape_ = new osg::Geode;

        };

        std::vector<OSGobj*> osgobj_;
};

#endif // SPOPENSCENEGRAPHGUI_H
















