#include <spirit/Gui/spOpenSceneGraphGui.h>


spOpenSceneGraphGui::spOpenSceneGraphGui()
{
    simtime_ = 0.0;
}

spOpenSceneGraphGui::~spOpenSceneGraphGui()
{}

void spOpenSceneGraphGui::InitGui()
{
    //viewer_.getCamera()->setViewMatrixAsLookAt( osg::Vec3(0.0f,-100.0f,0.0f), osg::Vec3(), osg::Z_AXIS );
    viewer_.setUpViewInWindow(20,20, 650, 650);
    viewer_.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer_.realize();
}

bool spOpenSceneGraphGui::ShouldQuit()
{
    return viewer_.done();
}
void spOpenSceneGraphGui::Refresh()
{
    viewer_.setSceneData(root_.get());
    viewer_.frame(simtime_);
    simtime_ += 0.001;
}
void spOpenSceneGraphGui::CheckKeyboardAction()
{
    //std::cout<<"CheckKeyboardAction() currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::AddBox(spBox& box)
{
    Eigen::Vector3d dim = box.GetDimensions();

    OSGobj* boxobj = new OSGobj;
    boxobj->name_ = "box";
    boxobj->translate_ = osg::Matrixd::translate(osg::Vec3(box.GetPose().translation()[0], box.GetPose().translation()[1], box.GetPose().translation()[2]));
    boxobj->rotx_ = osg::Matrix::rotate(box.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
    boxobj->roty_ = osg::Matrix::rotate(box.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
    boxobj->rotz_ = osg::Matrix::rotate(box.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));

    // vector is box center, the following are width, height, andd depth
    boxobj->shape_->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), dim[0], dim[1], dim[2]));
    boxobj->shape_->setColor(osg::Vec4(box.GetColor()[0], box.GetColor()[1], box.GetColor()[2], 1.0f));
    boxobj->geoshape_->addDrawable(boxobj->shape_.get());

    boxobj->transform_->setMatrix(boxobj->rotx_ * boxobj->roty_ * boxobj->rotz_ * boxobj->translate_);
    boxobj->transform_->addChild(boxobj->geoshape_.get());

    osgobj_.push_back(boxobj);
    box.SetGuiIndex(osgobj_.size()-1);
    root_->addChild(boxobj->transform_.get());
}

void spOpenSceneGraphGui::AddWaypoint(spWaypoint& waypoint)
{
    OSGobj* waypts = new OSGobj;
    waypts->name_ ="waypoint";
    waypts->translate_ = osg::Matrixd::translate(osg::Vec3(waypoint.GetPose().translation()[0], waypoint.GetPose().translation()[1], waypoint.GetPose().translation()[2]));
    waypts->rotx_ = osg::Matrix::rotate(waypoint.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
    waypts->roty_ = osg::Matrix::rotate(waypoint.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
    waypts->rotz_ = osg::Matrix::rotate(waypoint.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));

    waypts->shape_->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), 0.25, 0.25, 0.25));
    waypts->shape_->setColor(osg::Vec4(waypoint.GetColor()[0], waypoint.GetColor()[1], waypoint.GetColor()[2], 1.0f));
    waypts->geoshape_->addDrawable(waypts->shape_.get());

    waypts->transform_->setMatrix(waypts->rotx_ * waypts->roty_ * waypts->rotz_ * waypts->translate_);
    waypts->transform_->addChild(waypts->geoshape_.get());

    osgobj_.push_back(waypts);
    waypoint.SetGuiIndex(osgobj_.size()-1);
    root_->addChild(waypts->transform_.get());

    // normal vectors
    double vecdist = 0.25;
    osg::Vec3 start(waypoint.GetPose().translation()[0], waypoint.GetPose().translation()[1], waypoint.GetPose().translation()[2]);
    osg::Vec3 xf(waypoint.GetPose().translation()[0] + vecdist, waypoint.GetPose().translation()[1], waypoint.GetPose().translation()[2]);
    osg::ref_ptr<osg::Vec3Array> xpts = new osg::Vec3Array;
    xpts->push_back( start );
    xpts->push_back( xf );
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back( osg::Vec3(0.0f,-1.0f, 0.0f) );
    osg::ref_ptr<osg::Vec4Array> xcolor = new osg::Vec4Array;
    xcolor->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
    osg::ref_ptr<osg::Geometry> xline = new osg::Geometry;
    xline->setVertexArray(xpts.get());
    xline->setNormalArray(normals.get());
    xline->setNormalBinding(osg::Geometry::BIND_OVERALL);
    xline->setColorArray(xcolor.get());
    xline->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    xline->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
    root_->addChild(xline.get());

    osg::Vec3 yf(waypoint.GetPose().translation()[0], waypoint.GetPose().translation()[1] + vecdist, waypoint.GetPose().translation()[2]);
    osg::ref_ptr<osg::Vec3Array> ypts = new osg::Vec3Array;
    ypts->push_back( start );
    ypts->push_back( yf );
    osg::ref_ptr<osg::Vec4Array> ycolor = new osg::Vec4Array;
    ycolor->push_back(osg::Vec4(0.0,1.0,0.0,1.0));
    osg::ref_ptr<osg::Geometry> yline = new osg::Geometry;
    yline->setVertexArray(ypts.get());
    yline->setNormalArray(normals.get());
    yline->setNormalBinding(osg::Geometry::BIND_OVERALL);
    yline->setColorArray(ycolor.get());
    yline->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    yline->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
    root_->addChild(yline.get());

    osg::Vec3 zf(waypoint.GetPose().translation()[0], waypoint.GetPose().translation()[1], waypoint.GetPose().translation()[2] + vecdist);
    osg::ref_ptr<osg::Vec3Array> zpts = new osg::Vec3Array;
    zpts->push_back( start );
    zpts->push_back( zf );
    osg::ref_ptr<osg::Vec4Array> zcolor = new osg::Vec4Array;
    zcolor->push_back(osg::Vec4(0.0,0.0,1.0,1.0));
    osg::ref_ptr<osg::Geometry> zline = new osg::Geometry;
    zline->setVertexArray(zpts.get());
    zline->setNormalArray(normals.get());
    zline->setNormalBinding(osg::Geometry::BIND_OVERALL);
    zline->setColorArray(zcolor.get());
    zline->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    zline->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
    root_->addChild(zline.get());

    // velocity vector
    osg::Vec3 vel(waypoint.GetLinearVelocityInWorld()[0], waypoint.GetLinearVelocityInWorld()[1], waypoint.GetLinearVelocityInWorld()[2]);
    osg::ref_ptr<osg::Vec3Array> vpts = new osg::Vec3Array;
    vpts->push_back( start );
    vpts->push_back( vel );
    osg::ref_ptr<osg::Vec4Array> velcolor = new osg::Vec4Array;
    velcolor->push_back(osg::Vec4(1.0,1.0,1.0,1.0));
    osg::ref_ptr<osg::Geometry> vline = new osg::Geometry;
    vline->setVertexArray(vpts.get());
    vline->setNormalArray(normals.get());
    vline->setNormalBinding(osg::Geometry::BIND_OVERALL);
    vline->setColorArray(velcolor.get());
    vline->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    vline->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
    root_->addChild(vline.get());

}
void spOpenSceneGraphGui::AddVehicle(spVehicle& vehicle)
{
    OSGobj* bikeparts = new OSGobj;
    bikeparts->name_ = "frame";
    bikeparts->translate_ = osg::Matrixd::translate(osg::Vec3(vehicle.GetPose().translation()[0], vehicle.GetPose().translation()[1], vehicle.GetPose().translation()[2]));
    bikeparts->rotx_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
    bikeparts->roty_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
    bikeparts->rotz_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));

    Eigen::Vector3d framedim = vehicle.GetChassisSize();
    bikeparts->shape_->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), framedim[0], framedim[1], framedim[2]));
    bikeparts->shape_->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    bikeparts->geoshape_->addDrawable(bikeparts->shape_.get());

    bikeparts->transform_->setMatrix(bikeparts->rotx_ * bikeparts->roty_ * bikeparts->rotz_ * bikeparts->translate_);
    bikeparts->transform_->addChild(bikeparts->geoshape_.get());

    osgobj_.push_back(bikeparts);
    vehicle.SetGuiIndex(osgobj_.size()-1);
    root_->addChild(bikeparts->transform_.get());

    for(int ii=0; ii<vehicle.GetNumberOfWheels(); ii++) {
        OSGobj* bikeparts = new OSGobj;
        bikeparts->name_ = "wheel";
        bikeparts->translate_ = osg::Matrixd::translate(osg::Vec3(vehicle.GetWheel(ii)->GetPose().translation()[0], vehicle.GetWheel(ii)->GetPose().translation()[1], vehicle.GetWheel(ii)->GetPose().translation()[2]));
        bikeparts->rotx_ = osg::Matrix::rotate(vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
        bikeparts->roty_ = osg::Matrix::rotate(SP_PI/2 + vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
        bikeparts->rotz_ = osg::Matrix::rotate(SP_PI/2 + vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));

        bikeparts->shape_->setShape(new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f), vehicle.GetWheel(ii)->GetRadius(), vehicle.GetWheel(ii)->GetWidth()));
        bikeparts->shape_->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        bikeparts->geoshape_->addDrawable(bikeparts->shape_.get());

        bikeparts->transform_->setMatrix(bikeparts->rotx_ * bikeparts->roty_ * bikeparts->rotz_ * bikeparts->translate_);
        bikeparts->transform_->addChild(bikeparts->geoshape_.get());

        osgobj_.push_back(bikeparts);
        vehicle.GetWheel(ii)->SetGuiIndex(osgobj_.size()-1);
        root_->addChild(bikeparts->transform_.get());
    }
}

void spOpenSceneGraphGui::AddLineStrip(spLineStrip& linestrip)
{
    std::cout<<"AddLineStrip(spLineStrip& linestrip) currently has no functionality"<<std::endl;

}

void spOpenSceneGraphGui::AddMesh(spMesh& mesh){
    OSGobj* meshptr = new OSGobj;
    meshptr->name_ = "mesh";
    osgobj_.push_back(meshptr);
    mesh.SetGuiIndex(osgobj_.size()-1);
    root_->addChild(mesh.GetMesh().get());
}

void spOpenSceneGraphGui::UpdateBoxGuiObject(spBox& spobj)
{
   int gui_index = spobj.GetGuiIndex();
   //std::cout<<"The gui index is: "<<gui_index<<std::endl;

   if((!(spobj.GetGuiIndex() < osgobj_.size())) || (osgobj_[gui_index]->name_.compare("box"))){
     SPERROREXIT("gui object doesn't match spobject.");
    }

   osgobj_[gui_index]->translate_ = osg::Matrixd::translate(osg::Vec3(spobj.GetPose().translation()[0], spobj.GetPose().translation()[1], spobj.GetPose().translation()[2]));
   osgobj_[gui_index]->rotx_ = osg::Matrix::rotate(spobj.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
   osgobj_[gui_index]->roty_ = osg::Matrix::rotate(spobj.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
   osgobj_[gui_index]->rotz_ = osg::Matrix::rotate(spobj.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));
   osgobj_[gui_index]->transform_->setMatrix(osgobj_[gui_index]->rotx_ * osgobj_[gui_index]->roty_ * osgobj_[gui_index]->rotz_ * osgobj_[gui_index]->translate_);

}

void spOpenSceneGraphGui::UpdateVehicleGuiObject(spVehicle& vehicle)
{
   int chassis_index = vehicle.GetGuiIndex();
   if((!(vehicle.GetGuiIndex() < osgobj_.size())) || (osgobj_[chassis_index]->name_.compare("frame"))){
     SPERROREXIT("gui object doesn't match spobject.");
   }
   osgobj_[chassis_index]->translate_ = osg::Matrixd::translate(osg::Vec3(vehicle.GetPose().translation()[0], vehicle.GetPose().translation()[1], vehicle.GetPose().translation()[2]));
   osgobj_[chassis_index]->rotx_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
   osgobj_[chassis_index]->roty_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
   osgobj_[chassis_index]->rotz_ = osg::Matrix::rotate(vehicle.GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));
   osgobj_[chassis_index]->transform_->setMatrix(osgobj_[chassis_index]->rotx_ * osgobj_[chassis_index]->roty_ * osgobj_[chassis_index]->rotz_ * osgobj_[chassis_index]->translate_);

   for(int ii=0; ii<vehicle.GetNumberOfWheels(); ii++) {
     int wheel_index = vehicle.GetWheel(ii)->GetGuiIndex();
     osgobj_[wheel_index]->translate_ = osg::Matrixd::translate(osg::Vec3(vehicle.GetWheel(ii)->GetPose().translation()[0], vehicle.GetWheel(ii)->GetPose().translation()[1], vehicle.GetWheel(ii)->GetPose().translation()[2]));
     osgobj_[wheel_index]->rotx_ = osg::Matrix::rotate(vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[0], osg::Vec3(1.0, 0.0, 0.0));
     osgobj_[wheel_index]->roty_ = osg::Matrix::rotate(SP_PI/2 + vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[1], osg::Vec3(0.0, 1.0, 0.0));
     osgobj_[wheel_index]->rotz_ = osg::Matrix::rotate(SP_PI/2 + vehicle.GetWheel(ii)->GetPose().rotation().eulerAngles(0,1,2)[2], osg::Vec3(0.0, 0.0, 1.0));
     osgobj_[wheel_index]->transform_->setMatrix(osgobj_[wheel_index]->rotx_ * osgobj_[wheel_index]->roty_ * osgobj_[wheel_index]->rotz_ * osgobj_[wheel_index]->translate_);
   }
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
            //std::cout<<"WAYPOINT currently no update functionality."<<std::endl;
            break;
          }
          case spObjectType::BOX:
          {
            UpdateBoxGuiObject((spBox&)spobj.GetObject(ii));
            //SPERROREXIT("BOX object not in use.");
            break;
          }
          case spObjectType::VEHICLE_BIKE:
          {
            UpdateVehicleGuiObject((spVehicle&)spobj.GetObject(ii));
            //SPERROREXIT("VEHICLE_AWSD not is use.");
            break;
            }
          case spObjectType::LINESTRIP:
          {
            std::cout<<"LINESTRIP currently no update functionality."<<std::endl;
            break;
          }
          case spObjectType::MESH:
          {
            //std::cout<<"MESH currently no update functionality."<<std::endl;
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
    //std::cout<<"UpdateSpiritObjectsFromGui(Objects& spobjects) currently has no functionality"<<std::endl;

}
void spOpenSceneGraphGui::RemoveVehicle(spVehicle& vehicle)
{
    std::cout<<"RemoveVehicle(spVehicle& vehicle) currently has no functionality"<<std::endl;

}
