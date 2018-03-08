#ifndef CARSIMFUNCTOR_H__
#define CARSIMFUNCTOR_H__
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>

class CarSimFunctor {
 public:
  CarSimFunctor(
      const spVehicleConstructionInfo& info, const spState& initial_state,
      Gui* gui = nullptr /*, const std::shared_ptr<Objects>& objects=nullptr*/)
      : vehicle_info_(info),
        initial_state_(initial_state),
        gui_(gui),
        thread_(nullptr) /*, objects_(objects)*/ {
    objects_ = std::make_shared<Objects>();
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0, 0, -0.5));
    gnd_handle_ = objects_->CreateBox(gnd_pose_, spBoxSize(20, 20, 1), 0,
                                      spColor(1, 0, 0));
    car_handle_ = objects_->CreateVehicle(vehicle_info_);
    if (initial_state_.substate_vec.size() != info.wheels_anchor.size()) {
      SPERROR("Provided state does not match the VehicleInfo");
    }
    //    if(initial_state_.substate_vec.size() == 0) {
    //      // wheel initialization has not been requested. create wheels with
    //      default state values (check spTypes.h)
    //      for(int ii = 0; ii<info.wheels_anchor.size(); ii++) {
    //        initial_state_.InsertSubstate();
    //      }
    //    }
    //    for(int ii = 0; ii<initial_state.substate_vec.size(); ii++) {
    //      initial_state_.InsertSubstate();
    //    }
    ((spAWSDCar&)objects_->GetObject(car_handle_)).SetState(initial_state_);
    if ((gui_ != nullptr)) {
      gui_->AddObject(objects_->GetObject(car_handle_));
    }
  }

  ~CarSimFunctor() {
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

  CarSimFunctor(const CarSimFunctor& obj) : vehicle_info_(obj.vehicle_info_) {
    SPERROR("cpy constructor called. AVOID calling cpyConstructors");
  }

  void RunInThread(int thread_id, double num_sim_steps, double step_size,
                   const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                   int pert_index,
                   std::shared_ptr<spStateSeries> traj_states = nullptr,
                   std::shared_ptr<spState> init_state = nullptr,
                   std::shared_ptr<double> cost = nullptr,
			std::shared_ptr<double> tire_friction = nullptr ) {
    thread_ = std::make_unique<std::thread>(
        &CarSimFunctor::operator(), this, thread_id, num_sim_steps, step_size,
        cntrl_vars, epsilon, pert_index, traj_states,init_state,cost,tire_friction);
  }

  void WaitForThreadJoin() {
    thread_->join();
    thread_.reset();
  }

  void operator()(int thread_id, double num_sim_steps, double step_size,
                  const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                  int pert_index,
                  std::shared_ptr<spStateSeries> traj_states = nullptr,
                  std::shared_ptr<spState> init_state = nullptr,
                  std::shared_ptr<double> cost = nullptr,
			std::shared_ptr<double> tire_friction = nullptr ) {
    double radius = 1;
    double total_cost = 0;
    spBox& gnd = (spBox&)objects_->GetObject(gnd_handle_);
    gnd.SetFriction(1);
    spAWSDCar& car = (spAWSDCar&)objects_->GetObject(car_handle_);
     if(tire_friction != nullptr) {
       car.UpdateWheelFriction(*tire_friction);
     }
    if(init_state != nullptr) {
      car.SetState(*init_state);
    }
    spCurve control_curve(2, 2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
    //    spPointXd curvature_cost(2);
    //    control_curve.Get2ndDrivativeCurveArea(curvature_cost);
    if (pert_index >= 0) {
      control_curve.PerturbControlPoint(
          pert_index, epsilon /**std::abs(cntrl_vars.data()[pert_index])*/);
    }
    if (traj_states != nullptr) {
      traj_states->push_back(std::make_shared<spState>(car.GetState()));
    }
    for (int ii = 0; ii < num_sim_steps; ii++) {
      control_curve.GetPoint(sample_control, ii / (double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineMaxVel(sample_control[1]);
//      car.SetEngineMaxVel(100);
//      car.SetEngineTorque(sample_control[1]*0.00001);
      objects_->StepPhySimulation(step_size);
      if ((gui_ != nullptr)) {
        gui_->Iterate(*objects_);
        spGeneralTools::Delay_ms(1000 * step_size);
      }
      if (traj_states != nullptr) {
        traj_states->push_back(std::make_shared<spState>(car.GetState()));
      }

      if (cost != nullptr) {
	double index_dif = 1;//(num_sim_steps-ii);
        spTranslation position = car.GetState().pose.translation();
        position[2] = 0;
        double curr_radius = position.norm();
        total_cost += index_dif*index_dif*std::abs(radius-curr_radius);
      }
    }
    *cost = total_cost;
  }

  const spState& GetState() {
    return ((spAWSDCar&)objects_->GetObject(car_handle_)).GetState();
  }

//  void SetState(const spState& state) {
//    spAWSDCar& car = (spAWSDCar&)objects_->GetObject(car_handle_);
//    car.SetState(state);
//  }

 public:
  const spVehicleConstructionInfo& vehicle_info_;
  spCtrlPts3ord_3dof traj_curve_;
  std::shared_ptr<Objects> objects_;
  spObjectHandle gnd_handle_;
  spObjectHandle car_handle_;
  spState initial_state_;
  Gui* gui_;
  std::unique_ptr<std::thread> thread_;
};

#endif  // CARSIMFUNCTOR_H__
