#include <spirit/Objects.h>

Objects::Objects(){}
Objects::~Objects(){}

int Objects::CreateBox(const spPose& pose, const spBoxSize& size, const double mass,const spColor& color) {
  std::shared_ptr<spBox> a_box = std::make_shared<spBox>();
  a_box->SetDimensions(size);
  a_box->SetPose(pose);
  a_box->SetMass(mass);
  a_box->SetColor(color);
  objects_.push_back(a_box);
  return (objects_.size()-1);
}

int Objects::CreateWaypoint(const spPose& pose, const spColor& color) {
  std::shared_ptr<spWaypoint> a_waypoint = std::make_shared<spWaypoint>();
  a_waypoint->SetPose(pose);
  a_waypoint->SetColor(color);
  objects_.push_back(a_waypoint);
  return (objects_.size()-1);
}

int Objects::CreateVehicle(const spVehicleConstructionInfo& vehicle_info) {
  switch (vehicle_info.vehicle_type) {
    case spVehicleConfig::AWSD:
    {
      std::shared_ptr<spAWSDCar> a_vehicle = std::make_shared<spAWSDCar>(vehicle_info);
      objects_.push_back(a_vehicle);
      break;
    }
  }
  return (objects_.size()-1);
}

int Objects::CreateLineStrip(const spPose& pose, const spPoints3d& linestrip_pts, const spColor& color) {
  std::shared_ptr<spLineStrip> a_curve = std::make_shared<spLineStrip>();
  a_curve->SetPose(pose);
  a_curve->SetLineStripPoints(linestrip_pts);
  a_curve->SetColor(color);
  objects_.push_back(a_curve);
  return (objects_.size()-1);
}


void Objects::RemoveObj(int obj_index) {
  if(obj_index<objects_.size()) {
    objects_.erase(objects_.begin()+obj_index);
  } else {
    SPERROREXIT("Requested Object doesn't exist.");
  }
}

spCommonObject& Objects::GetObject(int obj_index) {
  if(obj_index<objects_.size()) {
    return *objects_[obj_index].get();
  } else {
    SPERROREXIT("Requested Object doesn't exist.");
  }
}

int Objects::GetNumOfObjects() {
  return objects_.size();
}
