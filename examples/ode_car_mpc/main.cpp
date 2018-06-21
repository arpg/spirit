#include <thread>
#include <spirit/spirit.h>
#include <atomic>

//#include <spirit/CarSimFunctorRK4.h>

int main(int argc, char** argv) {
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);
  spirit spworld(settings_obj);

  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);
  // create a flat ground
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);

  spState state;
  state.pose = spPose::Identity();
  state.pose.translate(spTranslation(1.1,0,0));
  Eigen::AngleAxisd rot1(0,Eigen::Vector3d::UnitZ());
  state.pose.rotate(rot1);
  state.linvel = spLinVel(0.0,0,0);
  state.rotvel = spRotVel(0,0,0);
  state.wheel_speeds = spWheelSpeedVec(0,0,0,0);

  car.SetState(state);

  /////////////////////////////////

  // set driving car's first pose

  // create a MPC controller with horizon
  float horizon = 2;
  spMPC mpc(spworld.car_param,horizon);

  spCtrlPts2ord_2dof controls;
  controls.col(0) = Eigen::Vector2d(0,1);
  controls.col(1) = Eigen::Vector2d(0,1);
  controls.col(2) = Eigen::Vector2d(0,1);

  while(1){
//    car.SetPose(posys_);
//    spPose ps = car.GetPose();
//    ps.translation()[2] = 0.07;
//    car.SetPose(car_init_pose);
//    car.SetState(sst);

    spTimestamp t0 = spGeneralTools::Tick();
    if(mpc.CircleManReg(state,controls,2,3)) {
      double calc_time = spGeneralTools::Tock_ms(t0);
      std::cout << "controls -> \n" << controls << std::endl;
      while(1);
//      spCurve controls_curve(2,2);
//      spPointXd next_control(2);
//      calc_time /= 1000;
//      spworld.objects_.StepPhySimulation(calc_time);
//      spworld.gui_.Iterate(spworld.objects_);
//      controls_curve.SetBezierControlPoints(controls);
      // only get the control signal for next point and apply to car
//      controls_curve.GetPoint(next_control,calc_time/horizon);
//      std::cout << "next signals are " << std::fixed << std::setprecision(3) << next_control.transpose() << std::endl;
//      car.SetFrontSteeringAngle(next_control[0]);
//      car.SetEngineMaxVel(/*next_control[1]*/100);
//      car.SetEngineTorque(next_control[1]*0.00001);
//      car.SetEngineTorque(2*0.00001);
//      if(flag_auto) {
//          if(next_control[1] > 30) {
//              next_control[1] = 30;
//          }
//          if(next_control[1] < -30) {
//              next_control[1] = -30;
//          }
//          commandMSG.set_steering_angle(-next_control[0]);
//          commandMSG.set_throttle_percent(gamepad_throttle/*next_control[1]*/);
//      } else {
//          commandMSG.set_steering_angle(gamepad_steering);
//          commandMSG.set_throttle_percent(gamepad_throttle);
//      }

//      ninja_car.UpdateCarCommand(commandMSG);
//      controls.col(0) = next_control;
      //double calc_time = spGeneralTools::Tock_ms(t0);
      //std::cout << "calc time was " << calc_time << std::endl;

//      spworld.objects_.StepPhySimulation(DISCRETIZATION_STEP_SIZE);
      spworld.gui_.Iterate(spworld.objects_);

//    ninja_car.UpdateCarCommand(commandMSG);
//    car.SetPose(posys_);
//    spworld.gui_.Iterate(spworld.objects_);
//    car.SetEngineMaxVel(50);
//    car.SetFrontSteeringAngle(0);
//    for(int ii=0; ii<20; ii++) {
//        spworld.objects_.StepPhySimulation(0.1);
//        spworld.gui_.Iterate(spworld.objects_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
