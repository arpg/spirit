#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>
#include <atomic>


hal::CarCommandMsg commandMSG;
double gamepad_steering = 0;
double gamepad_throttle = 0;
spPose posys_;
bool flag_auto = false;

void Posys_Handler(hal::PoseMsg& PoseData) {
    posys_ = spPose::Identity();
    posys_.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0.06/*PoseData.pose().data(2)*/));
    spRotation rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
    Eigen::AngleAxisd tracker_rot(-SP_PI_HALF,Eigen::Vector3d::UnitZ());
    posys_.rotate(rot);
    posys_.rotate(tracker_rot);
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  gamepad_steering = -_msg.axes().data(0);
  gamepad_throttle = _msg.axes().data(4)*30;
  if(_msg.buttons().data(5)){
      flag_auto = true;
  } else {
      flag_auto = false;
  }
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
//              << msg.whflag_autoeel_speed_rr() << ", "
//              << -msg.steer_angle() << ", "
//              << std::endl;
}

std::atomic<double> st0;
std::atomic<double> st1;
std::atomic<double> st2;
std::atomic<double> ac0;
std::atomic<double> ac1;
std::atomic<double> ac2;

std::atomic<bool> gFlag;
std::atomic<double> gCurrStr;
std::atomic<double> gCurrAcc;

void ApplyCommand() {
//    hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");
    //ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);
  spCurve curve(2,2);
  spCtrlPts2ord_2dof controls;
  spPointXd next_control(2);
  spTimestamp t0;
  double timer;
  double horizon = 0.6;
  while(1){
    if(gFlag) {
      gFlag = false;
      timer = 0;
      controls.col(0) = Eigen::Vector2d(st0,ac0);
      controls.col(1) = Eigen::Vector2d(st1,ac1);
      controls.col(2) = Eigen::Vector2d(st2,ac2);
      curve.SetBezierControlPoints(controls);
      t0 = spGeneralTools::Tick();
      curve.GetPoint(next_control,0);
    } else {
      timer = spGeneralTools::Tock_ms(t0)/1000;
      if(timer<horizon) {
        curve.GetPoint(next_control,timer/horizon);
      } else {
        curve.GetPoint(next_control,1);
      }
    }
    gCurrStr = next_control[0];
    gCurrAcc = next_control[1];

    if(flag_auto) {
        if(next_control[1] > 30) {
            next_control[1] = 30;
        }
        if(next_control[1] < -30) {
            next_control[1] = -30;
        }
        commandMSG.set_steering_angle(-next_control[0]);
        commandMSG.set_throttle_percent(gamepad_throttle/*next_control[1]*/);
    } else {
        commandMSG.set_steering_angle(gamepad_steering);
        commandMSG.set_throttle_percent(gamepad_throttle);
    }
    //  ninja_car.UpdateCarCommand(commandMSG);
  }

}

int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
//  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
//  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");

  //ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  //////////////////////////////
//  hal::Posys vicon("vicon://tracker:[ninja]");
//  vicon.RegisterPosysDataCallback(&Posys_Handler);
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

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);

  /////////////////////////////////
  spTrajectory traj(spworld.gui_,spworld.objects_);
  // put waypoints on a elliptical path
  double a = 1.5;
  double b = 1.5;
  int num_waypoints = 8;
  for(int ii=0; ii<num_waypoints; ii++) {
    // calculate ellipse radius from theta and then get x , y coordinates of ellipse from r and theta
    double theta = ii*(2*SP_PI)/num_waypoints;
    double r = (a*b)/sqrt(b*b*pow(cos(theta),2)+a*a*pow(sin(theta),2));
    double x = r*cos(theta);
    double y = r*sin(theta);
    // slope of the line is
    double angle = atan2(-(x*b*b),(y*a*a));
    spPose pose(spPose::Identity());
    pose.translate(spTranslation(x,y,0.07));
    Eigen::AngleAxisd rot(angle+SP_PI_HALF/*+0.6*/,Eigen::Vector3d::UnitZ());
    pose.rotate(rot);
    traj.AddWaypoint(pose,1);
    spRotVel rotvel(0,0,2);
    traj.GetWaypoint(ii).SetRotVel(rotvel);
    traj.GetWaypoint(ii).SetLinearVelocityDirection(spLinVel(0,1,0));
  }
  spworld.gui_.Iterate(spworld.objects_);
  traj.IsLoop(true);

  spLocalPlanner localplanner(spworld.car_param,false,&spworld.gui_);
  spBVPWeightVec weight_vec;
  weight_vec << 100, 100, 0, 0, 0, 10, 0.009, 0.009, 0.009, 0.01, 0.01, 0.01,0.1;
  localplanner.SetCostWeight(weight_vec);
  for(int ii=0; ii<traj.GetNumWaypoints(); ii++) {
    traj.SetTravelDuration(ii,0.5);
    localplanner.SolveInitialPlan(traj,ii);
    localplanner.SolveLocalPlan(traj,ii);
    spworld.gui_.Iterate(spworld.objects_);
  }


  // set driving car's first pose
  spPose car_init_pose(traj.GetWaypoint(0).GetPose());
  car_init_pose.translate(spTranslation(0,0,0));
  car.SetPose(car_init_pose);


  // create a MPC controller with horizon
  float horizon = 0.6;
  spMPC mpc(spworld.car_param,horizon);
  //spMPC mpc2(spworld.car_param,horizon);

  spCtrlPts2ord_2dof controls;

  spState sst = car.GetState();

  controls.col(0) = Eigen::Vector2d(0,10);
  controls.col(1) = Eigen::Vector2d(0,10);
  controls.col(2) = Eigen::Vector2d(0,10);

  std::thread applycommand_thread(ApplyCommand);

  while(1){
//    car.SetPose(posys_);
//    spPose ps = car.GetPose();
//    ps.translation()[2] = 0.07;
//    car.SetPose(car_init_pose);
//    car.SetState(sst);

    spTimestamp t0 = spGeneralTools::Tick();
    if(mpc.CalculateControls(traj,car.GetState(),controls)) {
      st0 = controls.col(0)[0];
      ac0 = controls.col(0)[1];
      st1 = controls.col(1)[0];
      ac1 = controls.col(1)[1];
      st2 = controls.col(2)[0];
      ac2 = controls.col(2)[1];
      gFlag = true;
      double calc_time = spGeneralTools::Tock_ms(t0);
      std::cout << "calc time was " << calc_time << std::endl;
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
    } else {
      SPERROREXIT("No Controls could be calculated! ");
    }

//    ninja_car.UpdateCarCommand(commandMSG);
//    car.SetPose(posys_);
//    spworld.gui_.Iterate(spworld.objects_);
//    car.SetEngineMaxVel(50);
//    car.SetFrontSteeringAngle(0);
//    for(int ii=0; ii<20; ii++) {
//        spworld.objects_.StepPhySimulation(0.1);
//        spworld.gui_.Iterate(spworld.objects_);
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  applycommand_thread.join();
  return 0;
}
