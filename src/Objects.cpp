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

void Objects::RemoveObj(int obj_index) {
  objects_.erase(objects_.begin()+obj_index);
}

spCommonObject& Objects::GetObject(int obj_index) {
  return *objects_[obj_index].get();
}

int Objects::GetNumOfObjects() {
  return objects_.size();
}
