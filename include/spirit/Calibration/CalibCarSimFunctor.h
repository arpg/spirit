#ifndef CALIBCARSIMFUNCTOR_H__
#define CALIBCARSIMFUNCTOR_H__
#include <spirit/Types/ctpl_stl.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>

class CalibCarSimFunctor {
 public:
  CalibCarSimFunctor(
      const spVehicleConstructionInfo& vehicle_params,double wheel_base,double friction, Gui* gui = nullptr)
      : vehicle_params_(vehicle_params), gui_(gui) {
    // check if perturbation is required on some spVehicleConstructionInfo parameters
    // perturb wheel_base
    vehicle_params_.wheels_anchor[0][1] = wheel_base/2;
    vehicle_params_.wheels_anchor[1][1] = -wheel_base/2;
    vehicle_params_.wheels_anchor[2][1] = -wheel_base/2;
    vehicle_params_.wheels_anchor[3][1] = wheel_base/2;
    vehicle_params_.wheel_friction = friction;
    thread_ = nullptr;
    objects_ = std::make_shared<Objects>();
    car_handle_ = objects_->CreateVehicle(vehicle_params_);
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0, 0, -0.5));
    gnd_handle_ = objects_->CreateBox(gnd_pose_, spBoxSize(50, 50, 1), 0,
                                      spColor(1, 0, 0));
    spBox& gnd = (spBox&)objects_->GetObject(gnd_handle_);
    gnd.SetFriction(1);
    gnd.SetRollingFriction(0.1);

    if ((gui_ != nullptr)) {
      gui_->AddObject(objects_->GetObject(car_handle_));
    }
  }

  ~CalibCarSimFunctor() {
    if ((gui_ != nullptr)) {
      gui_->RemoveObject(objects_->GetObject(car_handle_));
    }
    objects_->RemoveObj(gnd_handle_);
    objects_->RemoveObj(car_handle_);
    if (thread_ != nullptr) {
      if (thread_->joinable()) {
        thread_->join();
      }
      thread_.reset();
    }
  }

  CalibCarSimFunctor(const CalibCarSimFunctor& obj) : vehicle_params_(obj.vehicle_params_) {
    SPERROREXIT("cpy constructor called. AVOID calling cpyConstructors");
  }

  void RunInThread(const spStateSeries& ref_states ,std::shared_ptr<spStateSeries> traj_states) {
    thread_ = std::make_unique<std::thread>(
        &CalibCarSimFunctor::operator(), this, ref_states, traj_states);
  }

  void WaitForThreadJoin() {
    thread_->join();
    thread_.reset();
  }

  void operator()(const spStateSeries& ref_states ,std::shared_ptr<spStateSeries> traj_states = nullptr) {
    spAWSDCar& car = (spAWSDCar&)objects_->GetObject(car_handle_);
    car.SetState(*(ref_states[0]));
    for (int ii = 0; ii < ref_states.size()-1; ii++) {
      car.SetFrontSteeringAngle(ref_states[ii]->current_controls.first);
      car.SetEngineMaxVel(ref_states[ii]->current_controls.second);
//      double travel_time_ms = spGeneralTools::TickTock_ms(ref_states[ii]->time_stamp,ref_states[ii+1]->time_stamp);
      double travel_time_ms = 100;
      objects_->StepPhySimulation(travel_time_ms*0.001);
      if ((gui_ != nullptr)) {
        gui_->Iterate(*objects_);
//        spGeneralTools::Delay_ms(travel_time_ms);
      }
      if(traj_states!= nullptr) {
        traj_states->push_back(std::make_shared<spState>(car.GetState()));
      }
    }
  }

 public:
  spVehicleConstructionInfo vehicle_params_;
  spCtrlPts3ord_3dof traj_curve_;
  std::shared_ptr<Objects> objects_;
  spObjectHandle gnd_handle_;
  spObjectHandle car_handle_;
  Gui* gui_;
  std::unique_ptr<std::thread> thread_;
};

#endif  // CALIBCARSIMFUNCTOR_H__
