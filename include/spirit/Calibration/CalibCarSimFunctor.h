#ifndef CALIBCARSIMFUNCTOR_H__
#define CALIBCARSIMFUNCTOR_H__
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>

class CalibCarSimFunctor {
 public:
  CalibCarSimFunctor(std::shared_ptr<spVehicleConstructionInfo> vehicle_params,
                     int perturbation_index = 0, double epsilon = 0,Gui* gui = nullptr)
    : gui_(gui) {

    thread_ = nullptr;
    objects_ = std::make_shared<Objects>(spPhyEngineType::PHY_BULLET);
    // perturb parameters
    Eigen::VectorXd param_vec(vehicle_params->GetParameterVector());
    param_vec[perturbation_index] += epsilon;
    vehicle_params->SetParameterVector(param_vec);
    car_handle_ = objects_->CreateVehicle(*vehicle_params);
    param_vec[perturbation_index] -= epsilon;
    vehicle_params->SetParameterVector(param_vec);

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

  CalibCarSimFunctor(const CalibCarSimFunctor& obj) {
    SPERROREXIT("cpy constructor called. AVOID calling cpyConstructors");
  }

  void RunInThread(const spStateSeries& ref_states ,std::shared_ptr<spStateSeries> traj_states) {
    if(gui_ != nullptr) {
      SPERROREXIT("Can not use gui in multithreaded mode.");
    }
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
  spCtrlPts3ord_3dof traj_curve_;
  std::shared_ptr<Objects> objects_;
  spObjectHandle gnd_handle_;
  spObjectHandle car_handle_;
  Gui* gui_;
  std::unique_ptr<std::thread> thread_;
};

#endif  // CALIBCARSIMFUNCTOR_H__
