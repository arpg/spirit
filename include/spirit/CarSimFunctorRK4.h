#ifndef CARSIMFUNCTORRK4_H__
#define CARSIMFUNCTORRK4_H__
#include <iostream>
#include <spirit/Types/spTypes.h>
#include <spirit/spSettings.h>
#include <spirit/CarODE.h>
#include <spirit/RK4.h>
#include <spirit/CarODE.h>

class CarSimFunctorRK4 {
 public:
  CarSimFunctorRK4(
      const spVehicleConstructionInfo& info, const spState& initial_state)
      : initial_state_(initial_state),
        thread_(nullptr) {
  }

  ~CarSimFunctorRK4() {
    if (thread_ != nullptr) {
      if (thread_->joinable()) {
        thread_->join();
      }
      thread_.reset();
    }
  }

  CarSimFunctorRK4(const CarSimFunctorRK4& obj) : vehicle_info_(obj.vehicle_info_) {
    SPERROREXIT("cpy constructor called. AVOID calling cpyConstructors");
  }

  void RunInThread(int thread_id, double num_sim_steps, double step_size,
                   const spCtrlPts2ord_2dof& cntrl_vars, double epsilon,
                   int pert_index,
                   std::shared_ptr<spStateSeries> traj_states = nullptr,
                   std::shared_ptr<spState> init_state = nullptr,
                   std::shared_ptr<double> cost = nullptr,
      std::shared_ptr<double> tire_friction = nullptr ) {
    thread_ = std::make_unique<std::thread>(
        &CarSimFunctorRK4::operator(), this, thread_id, num_sim_steps, step_size,
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
                  std::shared_ptr<spState> init_state = nullptr ) {
    Eigen::ArrayXd init(10);
    RK4 rk4solver(0.01);
    rk4solver.RegisterODE(&CarODE);
    if(init_state != nullptr) {
      // rotate from inertial frame to body frame
      Eigen::Vector3d g_linvel = init_state->linvel;
      Eigen::Vector3d g_rotvel = ;

      init[0] = 0.001;
      init[1] = 0;
      init[2] = init_state->rotvel[2];
      init[3] = init_state->wheel_speeds[0];
      init[4] = init_state->wheel_speeds[1];
      init[5] = init_state->wheel_speeds[2];
      init[6] = init_state->wheel_speeds[3];
      init[7] = init_state->pose.translation()[0];
      init[8] = init_state->pose.translation()[1];
      //TODO resolve how to manage rotations
      init[9] = init_state->pose.rotation().eulerAngles(0,1,2)[2];
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
      Eigen::ArrayXXd traj = rk4solver.Solve(init,500);


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
	}

  const spState& GetState() {
    return ((spAWSDCar&)objects_->GetObject(car_handle_)).GetState();
  }

//  void SetState(const spState& state) {
//    spAWSDCar& car = (spAWSDCar&)objects_->GetObject(car_handle_);
//    car.SetState(state);
//  }

 public:
  spCtrlPts3ord_3dof traj_curve_;
  spState initial_state_;
  std::unique_ptr<std::thread> thread_;
};

#endif  // CARSIMFUNCTORRK4_H__
