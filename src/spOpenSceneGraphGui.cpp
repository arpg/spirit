#include <spirit/Gui/spOpenSceneGraphGui.h>


spOpenSceneGraphGui::spOpenSceneGraphGui()
{}

spOpenSceneGraphGui::~spOpenSceneGraphGui()
{}

void spOpenSceneGraphGui::InitGui()
{
    viewer_.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
    viewer_.setUpViewInWindow(20,20, 750, 750);
    viewer_.setCameraManipulator(new osgGA::TrackballManipulator());
}

bool spOpenSceneGraphGui::ShouldQuit()
{
    return viewer_.done();
}
void spOpenSceneGraphGui::Refresh()
{
    viewer_.setSceneData(root_.get());
    viewer_.realize();
}
void spOpenSceneGraphGui::CheckKeyboardAction()
{
    std::cout<<"CheckKeyboardAction() currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::AddBox(spBox& box)
{
    Eigen::Vector3d dim = box.GetDimensions();
    translatebox_ = osg::Matrixd::translate(osg::Vec3( box.GetPose().translation()[0], box.GetPose().translation()[1], box.GetPose().translation()[2] ));
    rotx_ = osg::Matrix::rotate( box.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0) );
    roty_ = osg::Matrix::rotate( box.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0) );
    rotz_ = osg::Matrix::rotate( box.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0) );

    osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable;
    // vector is box center, the following are width, height, andd depth
    shape->setShape( new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), (float)dim[0], (float)dim[1], (float)dim[2]) );
    shape->setColor( osg::Vec4((float)box.GetColor()[0], (float)box.GetColor()[1], (float)box.GetColor()[2], 1.0f) );
    osg::ref_ptr<osg::Geode> gbox = new osg::Geode;
    gbox->addDrawable(shape.get());

    transformbox_->setMatrix(rotx_*roty_*rotz_*translatebox_);
    transformbox_->addChild(gbox.get());
    root_->addChild(transformbox_);

}

void spOpenSceneGraphGui::AddWaypoint(spWaypoint& waypoint)
{
    std::cout<<"AddWaypoint(spWaypoint& waypoint) currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::AddVehicle(spVehicle& vehicle)
{
    std::cout<<"AddVehicle(spVehicle& vehicle) currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::AddLineStrip(spLineStrip& linestrip)
{
    std::cout<<"AddLineStrip(spLineStrip& linestrip) currently has no functionality"<<std::endl;

}

void spOpenSceneGraphGui::UpdateBoxGuiObject(spBox& spobj)
{
/*  int gui_index = spobj.GetGuiIndex();
  if((!(spobj.GetGuiIndex()<globjects_.size())) || (globjects_[gui_index]->ObjectName().compare("box"))){
    SPERROREXIT("gui object doesn't match spobject.");
  }
  globjects_[gui_index]->SetPose(spobj.GetPose().matrix());
  globjects_[gui_index]->SetScale(spobj.GetDimensions()); */

  translatebox_ = osg::Matrixd::translate(osg::Vec3( spobj.GetPose().translation()[0], spobj.GetPose().translation()[1], spobj.GetPose().translation()[2] ));
  rotx_ = osg::Matrix::rotate( spobj.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0) );
  roty_ = osg::Matrix::rotate( spobj.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0) );
  rotz_ = osg::Matrix::rotate( spobj.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0) );
  transformbox_->setMatrix(rotx_*roty_*rotz_*translatebox_);
}

void spOpenSceneGraphGui::UpdateGuiObjectsFromSpirit(Objects &spobj)
{
    // go through all spirit objects
    for(spObjectHandle ii=spobj.GetListBegin(); ii!=spobj.GetListEnd(); ++ii) {
      //only update objects which had gui property changes
      if(spobj.GetObject(ii).HasChangedGui() && (spobj.GetObject(ii).GetGuiIndex()!=-1)) {
        // update the gui object
        switch (spobj.GetObject(ii).GetObjecType()) {
          case spObjectType::WHEEL:
          {
            SPERROREXIT("WHEEL object should not be created by itself.");
            break;
          }
          case spObjectType::WAYPOINT:
          {
            SPERROREXIT("WAYPOINT object should not be created by itself.");
            break;
          }
          case spObjectType::BOX:
          {
            UpdateBoxGuiObject((spBox&)spobj.GetObject(ii));
            break;
          }
          case spObjectType::LINESTRIP:
          {
            SPERROREXIT("LINESTRIP object should not be created by itself.");
            break;
          }
          default:
          {
            SPERROREXIT("Unknown spirit object type.");
          }
        }
      }
    }
}


void spOpenSceneGraphGui::UpdateSpiritObjectsFromGui(Objects& spobjects)
{
    std::cout<<"UpdateSpiritObjectsFromGui(Objects& spobjects) currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::RemoveVehicle(spVehicle& vehicle)
{
    std::cout<<"RemoveVehicle(spVehicle& vehicle) currently has no functionality"<<std::endl;

}
