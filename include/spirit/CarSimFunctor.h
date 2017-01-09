#ifndef THREADPOOL_H__
#define THREADPOOL_H__
#include <spirit/Types/ctpl_stl.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/Physics.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>

class CarSimFunctor {
 public:
  CarSimFunctor(const spVehicleConstructionInfo& info, const spPhyEngineType& phy_type)
      : vehicle_info_(info), gnd_pose_(spPose::Identity()) {
    physics_.Create(phy_type);
//    std::cout << "CarSimFunctor Constructor !" << std::endl;
  }

  ~CarSimFunctor() {
//    std::cout << "D" << std::endl;
  }

  void operator()(int thread_id ) {
    // create ground at zero
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    int gnd_index = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
    physics_.AddObject(objects_.GetObject(gnd_index));
    obj_index_ = objects_.CreateVehicle(vehicle_info_);
    physics_.AddObject(objects_.GetObject(obj_index_));
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(obj_index_);
//    std::cout << "car pose was\n" << car.GetPose().matrix() << std::endl;
    car.SetEngineMaxVel(100);
    car.SetEngineTorque(10);
    car.SetSteeringServoMaxVel(100);
    car.SetSteeringServoTorque(10);
    car.SetClampToSurfaceFlag();
    car.SetFrontSteeringAngle(0);
    for(int ii=0;ii<5;ii++) {
      physics_.Iterate(objects_,0.5);
    }
//    std::cout << "car pose is\n" << car.GetPose().matrix() << std::endl;

//    std::cout << "operator done" << std::endl;

  }

 private:
  spVehicleConstructionInfo vehicle_info_;
  Physics physics_;
  Objects objects_;
  spPose gnd_pose_;
  int obj_index_;
};

#endif  // THREADPOOL_H__
