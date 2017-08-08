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
  CarSimFunctor(const spVehicleConstructionInfo& info ,const spState& initial_state,Gui* gui=nullptr/*, const std::shared_ptr<Objects>& objects=nullptr*/) : vehicle_info_(info), initial_state_(initial_state), gui_(gui)/*, objects_(objects)*/ {
    std::cout << "new object created " << std::endl;
    state_ = nullptr;
//    if(objects == nullptr) {
//      objects_ = std::make_shared<Objects>();
//    }
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
#if 0
    objects_ = std::make_shared<Objects>();
    gnd_handle_ = objects_->CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(1,0,0));
    car_handle_ = objects_->CreateVehicle(vehicle_info_);
    if(initial_state_.substate_vec.size() != info.wheels_anchor.size()) {
      SPERROR("Provided state does not match the VehicleInfo");
    }
#endif
//    if(initial_state_.substate_vec.size() == 0) {
//      // wheel initialization has not been requested. create wheels with default state values (check spTypes.h)
//      for(int ii = 0; ii<info.wheels_anchor.size(); ii++) {
//        initial_state_.InsertSubstate();
//      }
//    }
//    for(int ii = 0; ii<initial_state.substate_vec.size(); ii++) {
//      initial_state_.InsertSubstate();
//    }
#if 0
    ((spAWSDCar&)objects_->GetObject(car_handle_)).SetState(initial_state_);
    if((gui_!=nullptr)) {
      gui_->AddObject(objects_->GetObject(car_handle_));
    }
#endif
  }

  CarSimFunctor(const spVehicleConstructionInfo& info ) : vehicle_info_(info)/*, initial_state_(spState())*/ {
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
#if 1
    gnd_handle_ = objects_->CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
    car_handle_ = objects_->CreateVehicle(vehicle_info_);
#endif
  }

  ~CarSimFunctor() {
    std::cout << "Del"  << std::endl;
#if 0
    if((gui_!=nullptr)) {
      gui_->RemoveObject(objects_->GetObject(car_handle_));
    }
    objects_->RemoveObj(gnd_handle_);
    objects_->RemoveObj(car_handle_);
#endif
  }

  CarSimFunctor(const CarSimFunctor& obj) : vehicle_info_(obj.vehicle_info_){
//    SPERROR("cpy constructor called. AVOID calling cpyConstructors");
    std::cout << "cpy called" << std::endl;
    this->initial_state_ = obj.initial_state_;
  }

  void run(int thread_id,double num_sim_steps, double step_size, double epsilon, int pert_index) {
//    testth = new std::thread(&CarSimFunctor::operator (),this, thread_id,num_sim_steps,step_size,cntrl_vars,epsilon,pert_index,traj_states);
    thread_ = std::make_unique<std::thread>(&CarSimFunctor::blah,this, thread_id,num_sim_steps,step_size,epsilon,pert_index);
//    this->operator ()(thread_id,num_sim_steps,step_size,cntrl_vars,epsilon,pert_index,traj_states);
  }

  void WaitForThreadJoin(){
    thread_->join();
    thread_.reset();
  }


  void blah(int thread_id,double num_sim_steps, double step_size, double epsilon, int pert_index) {
    spVehicleConstructionInfo car_param;
    car_param.vehicle_type = spObjectType::VEHICLE_AWSD;
    car_param.pose = spPose::Identity();
    car_param.pose.translate(spTranslation(0,0,0.07));
    car_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
    car_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
    car_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
    car_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
    car_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
    car_param.cog = spTranslation(0, 0, 0);
    car_param.chassis_friction = 0;
    car_param.wheel_rollingfriction = 0.3;
    car_param.wheel_friction = 0.5;
    car_param.wheel_width = 0.04;
    car_param.wheel_radius = 0.057;
    car_param.susp_damping = 0;
    car_param.susp_stiffness = 10;
    car_param.susp_preloading_spacer = 0.1;
    car_param.susp_upper_limit = 0.013;
    car_param.susp_lower_limit = -0.028;
    car_param.wheel_mass = 0.1;
    car_param.chassis_mass = 5;
    car_param.steering_servo_lower_limit = -SP_PI / 4;
    car_param.steering_servo_upper_limit = SP_PI / 4;

    Objects objects_;
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(1,0,0));
    spObjectHandle car_handle = objects_.CreateVehicle(car_param);
    for(int ii=0;ii<num_sim_steps;ii++) {
      objects_.StepPhySimulation(step_size);
    }
    objects_.RemoveObj(gnd_handle);
    objects_.RemoveObj(car_handle);
  }




  void operator()(int thread_id,double num_sim_steps, double step_size,const spCtrlPts2ord_2dof& cntrl_vars, double epsilon, int pert_index, std::shared_ptr<spStateSeries> traj_states = nullptr) {
#if 0
    objects_ = std::make_shared<Objects>();
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    gnd_handle_ = objects_->CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(1,0,0));
    car_handle_ = objects_->CreateVehicle(vehicle_info_);
    for(int ii=0;ii<num_sim_steps;ii++) {
      objects_->StepPhySimulation(step_size);
    }
#if 0
    ((spAWSDCar&)objects_->GetObject(car_handle_)).SetState(initial_state_);

    ///////////////////////////////////////////////////////////////////////////////////
#if 1
    spBox& gnd = (spBox&) objects_->GetObject(gnd_handle_);
    gnd.SetFriction(1);
    spAWSDCar& car = (spAWSDCar&) objects_->GetObject(car_handle_);
//    car.SetPose(initial_state_.pose);
    car.SetState(initial_state_);
    car.SetEngineMaxVel(100);
    car.SetEngineTorque(100);
    car.SetSteeringServoMaxVel(100);
    car.SetSteeringServoTorque(100);
    car.SetRearSteeringAngle(0);
    spCurve control_curve(2,2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
//    spPointXd curvature_cost(2);
//    control_curve.Get2ndDrivativeCurveArea(curvature_cost);
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
#if 1
      objects_->StepPhySimulation(step_size);
#endif
      if((gui_!=nullptr)) {
        gui_->Iterate(*objects_);
        spGeneralTools::Delay_ms(1000*step_size);
      }
      if(traj_states != nullptr) {
        traj_states->push_back(car.GetState());
      }
    }
    state_ = car.GetState();
#endif
#endif
    objects_->RemoveObj(gnd_handle_);
    objects_->RemoveObj(car_handle_);
    objects_.reset();
#endif
  }

  const spState& GetState(){
    return *state_;
  }

  void SetState(const spState& state) {
#if 1
    spAWSDCar& car = (spAWSDCar&) objects_->GetObject(car_handle_);
    car.SetState(state);
#endif
  }

 public:
  const spVehicleConstructionInfo& vehicle_info_;
  spCtrlPts3ord_3dof traj_curve_;
//  spPoints3d traj_points_;
  std::shared_ptr<Objects> objects_;
  spObjectHandle gnd_handle_;
  spObjectHandle car_handle_;
  std::shared_ptr<spState> state_;
  spState initial_state_;
  Gui* gui_;
  std::unique_ptr<std::thread> thread_;
  volatile sig_atomic_t m_run_thread = true;
};

#endif  // THREADPOOL_H__
