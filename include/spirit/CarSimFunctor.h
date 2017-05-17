#ifndef THREADPOOL_H__
#define THREADPOOL_H__
#include <spirit/Types/ctpl_stl.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/Objects.h>
#include <spirit/Gui.h>

class CarSimFunctor {
 public:
  CarSimFunctor(const spVehicleConstructionInfo& info ,const spState& initial_state,Gui* gui=nullptr/*, Objects* objects=nullptr*/) : vehicle_info_(info), initial_state_(initial_state), gui_(gui)/*, objects_(objects)*/ {
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    gnd_handle_ = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(1,0,0));
    car_handle_ = objects_.CreateVehicle(vehicle_info_);
    if(initial_state_.substate_vec.size() != info.wheels_anchor.size()) {
      SPERROREXIT("Provided state does not match the VehicleInfo");
    }
//    if(initial_state_.substate_vec.size() == 0) {
//      // wheel initialization has not been requested. create wheels with default state values (check spTypes.h)
//      for(int ii = 0; ii<info.wheels_anchor.size(); ii++) {
//        initial_state_.InsertSubstate();
//      }
//    }
//    for(int ii = 0; ii<initial_state.substate_vec.size(); ii++) {
//      initial_state_.InsertSubstate();
//    }
    ((spAWSDCar&)objects_.GetObject(car_handle_)).SetState(initial_state_);
    if((gui_!=nullptr)) {
      gui_->AddObject(objects_.GetObject(car_handle_));
    }
  }

  CarSimFunctor(const spVehicleConstructionInfo& info ) : vehicle_info_(info)/*, initial_state_(spState())*/ {
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    gnd_handle_ = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
    car_handle_ = objects_.CreateVehicle(vehicle_info_);
  }

  ~CarSimFunctor() {
    if((gui_!=nullptr)) {
      gui_->RemoveObject(objects_.GetObject(car_handle_));
    }
    objects_.RemoveObj(gnd_handle_);
    objects_.RemoveObj(car_handle_);
  }

  void operator()(int thread_id,double num_sim_steps, double step_size,const spCtrlPts2ord_2dof& cntrl_vars, double epsilon, int pert_index, std::shared_ptr<spStateSeries> traj_states = nullptr) {
    spBox& gnd = (spBox&) objects_.GetObject(gnd_handle_);
    gnd.SetFriction(1);
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle_);

    car.SetPose(initial_state_.pose);
    car.SetEngineMaxVel(1000);
    car.SetEngineTorque(1000);
    car.SetSteeringServoMaxVel(1000);
    car.SetSteeringServoTorque(1000);
    car.SetFrontSteeringAngle(0);

    spCurve control_curve(2,2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
    spPointXd curvature_cost(2);
    control_curve.Get2ndDrivativeCurveArea(curvature_cost);
    if (pert_index >= 0) {
      control_curve.PerturbControlPoint(pert_index,epsilon/**std::abs(cntrl_vars.data()[pert_index])*/);
    }
    if(traj_states != nullptr) {
      traj_states->push_back(car.GetState());
    }
//    traj_points_.push_back(car.GetPose().translation());
    for(int ii=0;ii<num_sim_steps;ii++) {
      control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineMaxVel(sample_control[1]);
      objects_.StepPhySimulation(step_size);
      if((gui_!=nullptr)) {
        gui_->Iterate(objects_);
        spGeneralTools::Delay_ms(1000*step_size);
      }
//      traj_points_.push_back(car.GetPose().translation());
      if(traj_states != nullptr) {
        traj_states->push_back(car.GetState());
      }
    }
    state_ = car.GetState();
  }

  const spState& GetState(){
    return *state_;
  }

//  spPoints3d& GetTrajectoryPoints() {
//    return traj_points_;
//  }

 private:
  const spVehicleConstructionInfo& vehicle_info_;
  spCtrlPts3ord_3dof traj_curve_;
//  spPoints3d traj_points_;
  Objects objects_;
  spObjectHandle gnd_handle_;
  spObjectHandle car_handle_;
  std::shared_ptr<spState> state_;
  spState initial_state_;
  Gui* gui_;
};

#endif  // THREADPOOL_H__
