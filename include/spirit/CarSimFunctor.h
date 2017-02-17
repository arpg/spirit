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
  CarSimFunctor(const spVehicleConstructionInfo& info) : vehicle_info_(info) {
  }

  ~CarSimFunctor() {
    objects_.RemoveObj(car_handle);
    objects_.RemoveObj(gnd_handle);
  }

  void operator()(int thread_id,double num_sim_steps, double step_size,const spCtrlPts2ord_2dof& cntrl_vars, double epsilon, int pert_index) {
    spPose gnd_pose_ = spPose::Identity();
    gnd_pose_.translate(spTranslation(0,0,-0.5));
    gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(10,10,1),0,spColor(0,1,0));
    spBox& gnd = (spBox&) objects_.GetObject(gnd_handle);
    gnd.SetFriction(1);
    car_handle = objects_.CreateVehicle(vehicle_info_);
    spAWSDCar& car = (spAWSDCar&) objects_.GetObject(car_handle);
    car.SetEngineMaxVel(10);
    car.SetEngineTorque(10);
    car.SetSteeringServoMaxVel(10);
    car.SetSteeringServoTorque(10);
    car.SetFrontSteeringAngle(0);

    spCurve control_curve(2,2);
    control_curve.SetBezierControlPoints(cntrl_vars);
    spPointXd sample_control(2);
    spPointXd curvature_cost(2);
    control_curve.Get2ndDrivativeCurveArea(curvature_cost);
    if (pert_index >= 0) {
      control_curve.PerturbControlPoint(pert_index,epsilon/**std::abs(cntrl_vars.data()[pert_index])*/);
//    } else if (pert_index == 0) {
//      control_curve.PerturbControlPoint(pert_index,0.05*(epsilon/std::abs(epsilon)));
    }
    for(int ii=0;ii<num_sim_steps;ii++) {
      control_curve.GetPoint(sample_control,ii/(double)num_sim_steps);
      car.SetFrontSteeringAngle(sample_control[0]);
      car.SetEngineMaxVel(sample_control[1]);
      objects_.StepPhySimulation(step_size);
    }
    state_vec_ = car.GetStateVecor();
//    std::cout << "state is " << state_vec_.transpose() << std::endl;
  }

  const spStateVec& GetStateVec(){
    return state_vec_;
  }

 private:
  spVehicleConstructionInfo vehicle_info_;
  Objects objects_;
  spObjectHandle gnd_handle;
  spObjectHandle car_handle;
  spStateVec state_vec_;
};

#endif  // THREADPOOL_H__
