#include <spirit/Gui/spopenscenegraphgui.h>

spOpenSceneGraphGui::spOpenSceneGraphGui()
{}

spOpenSceneGraphGui()::~spOpenSceneGraphGui()
{}

void spOpenSceneGraphGui()::InitGui()
{
    viewer_.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
    viewer_.setUpViewInWindow(20,20, 750, 750);
    viewer_.setCameraManipulator(new osgGA::TrackballManipulator());
}

bool spOpenSceneGraphGui()::ShouldQuit()
{
}
void spOpenSceneGraphGui()::Refresh()
{}
void spOpenSceneGraphGui()::CheckKeyboardAction()
{}
void spOpenSceneGraphGui()::AddBox(spBox& box)
{}
void spOpenSceneGraphGui()::AddWaypoint(spWaypoint& waypoint)
{}
void spOpenSceneGraphGui()::AddVehicle(spVehicle& vehicle)
{}
void spOpenSceneGraphGui()::AddLineStrip(spLineStrip& linestrip)
{}
void spOpenSceneGraphGui()::UpdateGuiObjectsFromSpirit(Objects &spobj)
{}
void spOpenSceneGraphGui()::UpdateSpiritObjectsFromGui(Objects& spobjects)
{}
void spOpenSceneGraphGui()::RemoveVehicle(spVehicle& vehicle)
{}
