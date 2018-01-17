#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>


hal::CarCommandMsg commandMSG;
spPose posys_;

void Posys_Handler(hal::PoseMsg& PoseData) {
  posys_ = spPose::Identity();
  posys_.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),PoseData.pose().data(2)));
  spRotation rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  posys_.rotate(rot);
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  commandMSG.set_steering_angle(_msg.axes().data(0));
  commandMSG.set_throttle_percent(_msg.axes().data(5)*20);
}

void CarSensorCallback(hal::CarStateMsg msg) {
//    std::cout << "-> "
//              << msg.swing_angle_fl() << ", "
//              << msg.swing_angle_fr() << ", "
//              << msg.swing_angle_rl() << ", "
//              << msg.swing_angle_rr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_fr() << ", "
//              << msg.wheel_speed_fl() << ", "
//              << msg.wheel_speed_rl() << ", "
//              << msg.wheel_speed_rr() << ", "
//              << msg.steer_angle() << ", "
//              << std::endl;
}


int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
//  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");

//  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  //////////////////////////////
  hal::Posys vicon("vicon://tracker:[ninja]");
  vicon.RegisterPosysDataCallback(&Posys_Handler);
  /////////////////////////////
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

  /////////////////////////////////
//  spTrajectory traj(spworld.gui_,spworld.objects_);
//  // put waypoints on a elliptical path
//  double a = 2;
//  double b = 2;
//  int num_waypoints = 8;
//  for(int ii=0; ii<num_waypoints; ii++) {
//    // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
//    double theta = ii*(2*SP_PI)/num_waypoints;
//    double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
//    double x = r*cos(theta);
//    double y = r*sin(theta);
//    // slope of the line is
//    double angle = atan2(-(x*b*b),(y*a*a));
//    spPose pose(spPose::Identity());
//    pose.translate(spTranslation(x,y,0.07));
//    Eigen::AngleAxisd rot(angle+SP_PI_HALF,Eigen::Vector3d::UnitZ());
//    pose.rotate(rot);
//    traj.AddWaypoint(pose,4);
//  }
//  spworld.gui_.Iterate(spworld.objects_);
//  traj.IsLoop(true);

//  spLocalPlanner localplanner(spworld.car_param,&spworld.gui_);
//  spBVPWeightVec weight_vec;
//  weight_vec << 10, 10, 10, 0.1, 0.1, 1, 0.09, 0.09, 0.09, 0.1, 0.1, 0.1,0.1;
//  localplanner.SetCostWeight(weight_vec);
//  for(int ii=0; ii<traj.GetNumWaypoints(); ii++) {
//    traj.SetTravelDuration(ii,1);
//    localplanner.SolveInitialPlan(traj,ii);
//    localplanner.SolveLocalPlan(traj,ii,true);
//    spworld.gui_.Iterate(spworld.objects_);
//  }

//  // set driving car's first pose
//  spPose car_init_pose(traj.GetWaypoint(0).GetPose());
//  car_init_pose.translate(spTranslation(0,0,0));
//  car.SetPose(car_init_pose);

////  while(1){
////    gui_.Iterate(objects_);
////  }
//  spCtrlPts2ord_2dof controls;
//  controls.col(0) = Eigen::Vector2d(0,0);
//  controls.col(1) = Eigen::Vector2d(0,10);
//  controls.col(2) = Eigen::Vector2d(0,20);

//  // create a MPC controller with horizon
//  float horizon = 0.5;
//  spMPC mpc(spworld.car_param,horizon);

  while(1){
//    spTimestamp t0 = spGeneralTools::Tick();
//    if(mpc.CalculateControls(traj,car.GetState(),controls)) {
//      spCurve controls_curve(2,2);
//      spPointXd next_control(2);
//      controls_curve.SetBezierControlPoints(controls);
//      // only get the control signal for next point and apply to car
//      controls_curve.GetPoint(next_control,DISCRETIZATION_STEP_SIZE/horizon);
//      std::cout << "next signals are " << std::fixed << std::setprecision(3) << next_control.transpose() << std::endl;
//      car.SetFrontSteeringAngle(next_control[0]);
//      car.SetEngineMaxVel(next_control[1]);
//      controls.col(0) = next_control;
//      double calc_time = spGeneralTools::Tock_ms(t0);
//      std::cout << "calc time was " << calc_time << std::endl;

//      spworld.objects_.StepPhySimulation(DISCRETIZATION_STEP_SIZE);
//      spworld.gui_.Iterate(spworld.objects_);
//    } else {
//      SPERROREXIT("No Controls could be calculated! ");
//    }

//    ninja_car.UpdateCarCommand(commandMSG);
    car.SetPose(posys_);
    spworld.gui_.Iterate(spworld.objects_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
