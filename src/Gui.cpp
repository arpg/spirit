#include <spirit/Gui.h>

Gui::Gui(){

}

Gui::~Gui(){

}

void Gui::Create(const spGuiType gui_type) {
  switch(gui_type) {
    case spGuiType::GUI_NONE:
      std::cout << "Initializing spirit withoud gui" << std::endl;
      break;
    case spGuiType::GUI_PANGOSCENEGRAPH:
      gui_.reset();
      gui_ = std::make_shared<spPangolinScenegraphGui>();
      break;
    case spGuiType::GUI_OSG:
      gui_.reset();
      gui_ = std::make_shared<spOpenSceneGraphGui>();
      break;
    default:
      std::cerr << "Wrong type of GUI has been selected" << std::endl;
  }
  gui_->InitGui();
}

bool Gui::ShouldQuit() {
  return gui_->ShouldQuit();
}

void Gui::Refresh() {
  gui_->Refresh();
}
void Gui::CheckKeyboardAction() {
  gui_->CheckKeyboardAction();
}

//void Gui::RemoveObject(spCommonObject &obj) {

//}

void Gui::AddObject(spCommonObject &obj) {
   // std::cout<<obj.GetObjecType()<<std::endl;
  if(!gui_) {
    SPERROREXIT("gui_ object has not been created yet.");
  }
  switch(obj.GetObjecType()) {
    case spObjectType::WAYPOINT:
    {
      gui_->AddWaypoint((spWaypoint&) obj);
      break;
    }
    case spObjectType::BOX:
    {
      gui_->AddBox((spBox&) obj);
      break;
    }
    case spObjectType::VEHICLE_AWSD:
    {
      gui_->AddVehicle((spVehicle&) obj);
      break;
    }
    case spObjectType::VEHICLE_BIKE:
    {
      gui_->AddVehicle((spVehicle&) obj);
      break;
    }
    case spObjectType::LINESTRIP:
    {
      gui_->AddLineStrip((spLineStrip&) obj);
      break;
    }
    case spObjectType::MESH:
    {
      std::cout<<"before add mesh"<<std::endl;
      gui_->AddMesh((spMesh&) obj);
      break;
    }
    default:
    {
      std::cerr << "Unknown spirit object type." << std::endl;
    }
  }
}

void Gui::RemoveObject(spCommonObject &obj) {
  if(!gui_) {
    SPERROREXIT("gui_ object has not been created yet.");
  }
  if(obj.GetGuiIndex()==-1) {
    SPERROREXIT("object has never been added to gui");
  }
  switch(obj.GetObjecType()) {
    case spObjectType::WAYPOINT:
    {
//      gui_->AddWaypoint((spWaypoint&) obj);
      break;
    }
    case spObjectType::BOX:
    {
//      gui_->AddBox((spBox&) obj);
      break;
    }
    case spObjectType::VEHICLE_AWSD :
    {
      gui_->RemoveVehicle((spVehicle&) obj);
      break;
    }
    case spObjectType::LINESTRIP:
    {
//      gui_->AddLineStrip((spLineStrip&) obj);
      break;
    }
    default:
    {
      std::cerr << "Unknown spirit object type." << std::endl;
    }
  }
}

void Gui::Iterate(Objects& spobjects) {
  gui_->UpdateGuiObjectsFromSpirit(spobjects);
  gui_->Refresh();
  gui_->UpdateSpiritObjectsFromGui(spobjects);
  gui_->CheckKeyboardAction();
}
