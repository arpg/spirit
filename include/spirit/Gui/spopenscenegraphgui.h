#ifndef SPOPENSCENEGRAPHGUI_H
#define SPOPENSCENEGRAPHGUI_H

#include <spirit/spSettings.h>
#include <spirit/Gui/spCommonGui.h>
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>
#include <string>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/Node>
#include <osg/ref_ptr>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>

//// this class is the interface between OpenSceneGraph and spGui
class spOpenSceneGraphGui : public spCommonGUI
{
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
        void UpdateGuiObjectsFromSpirit(Objects &spobj);
        void UpdateSpiritObjectsFromGui(Objects& spobjects);
        void RemoveVehicle(spVehicle& vehicle);

    private:
        static void KeyActionMethodSample();
        void UpdateBoxGuiObject(spBox& spobj);
        void UpdateWaypointGuiObject(spWaypoint& spobj);
        void UpdateVehicleGuiObject(spVehicle& spobj);
        void UpdateLineStripGuiObject(spLineStrip& spobj);

        // TODO add variables relevent to OSG visualizer
        osgViewer::Viewer viewer_;

};

#endif // SPOPENSCENEGRAPHGUI_H
