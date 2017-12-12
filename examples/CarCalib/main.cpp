/// Sample Ccommandline Argument
// ./ninja_gui --groundmeshfile=/Users/saghli/code/datasets/meshes/lab.ply --paramfile=/Users/saghli/code/spirit/parameter_files/gui_params.csv

//#include <glog/logging.h>
#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Utils/GetPot>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <memory>
#include <HAL/Posys/PosysDevice.h>

#define SIM_CALIB

double steering_signal = 0;
double throttle_signal = 0;

hal::CarCommandMsg commandMSG;
spPose posys_;

void Posys_Handler(hal::PoseMsg& PoseData) {
  posys_ = spPose::Identity();
  posys_.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0.07/*PoseData.pose().data(2)*/));
  spRotation rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
  posys_.rotate(rot);
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  steering_signal = _msg.axes().data(0)*SP_PI_QUART;
  throttle_signal = -_msg.axes().data(4)*40;
//  std::cout << "steeromg command is " << steering_signal << std::endl;
//  std::cout << "throttle command is " << throttle_signal << std::endl;
}

int main(int argc, char** argv) {

  // get command line arguments
  GetPot cl_args(argc, argv);

//  bool has_ninjacar_ = cl_args.search("-car");
//  std::string car_uri = cl_args.follow("", "-car");

  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  hal::Posys vicon("vicon://192.168.20.100:[Ninja1]");
  vicon.RegisterPosysDataCallback(&Posys_Handler);

  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");

  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  spirit spworld(settings_obj);

#ifdef SIM_CALIB
  // create a simulated car
  spworld.car_param.wheel_friction = 0.75;
  spworld.car_param.wheel_rollingfriction = 0.1;
  spworld.car_param.engine_torque = 0.01;
  spworld.car_param.chassis_mass = 13;
  spworld.car_param.steering_servo_max_velocity = 5;
//  spworld.car_param.steering_servo_max_velocity = 1;
  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);
  //second car
  spObjectHandle car2_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car2_handle));
  spAWSDCar& car2 = (spAWSDCar&) spworld.objects_.GetObject(car2_handle);
  // create a flat ground
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));
  // set ground friction
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);

#endif


  ///////////////////////////
  /// testing new stuff

  ////////////////////////////

  spworld.car_param.wheel_friction = 0.5;
  double wheel_base = 0.38;
  spworld.car_param.wheels_anchor[0][1] = wheel_base/2;
  spworld.car_param.wheels_anchor[1][1] = -wheel_base/2;
  spworld.car_param.wheels_anchor[2][1] = -wheel_base/2;
  spworld.car_param.wheels_anchor[3][1] = wheel_base/2;

  unsigned int window_size = 5;
  unsigned int queue_size = 30;
  double batch_min_entropy = 10;
  // create a candidate_window and a priority queue
  PriorityQueue priority_queue(queue_size,spworld.car_param);
  // create a new candidate window
  CandidateWindow candidate_window(window_size,1,spworld.car_param/*,&spworld.gui_*/);
  EntropyTable entropytable(spworld.car_param,2,10);
//  candidate_window.prqueue_func_ptr_ = std::bind(&PriorityQueue::PushBackCandidateWindow,&priority_queue,std::placeholders::_1);
  candidate_window.prqueue_func_ptr_ = std::bind(&EntropyTable::PushBackCandidateWindow,&entropytable,std::placeholders::_1);
  int cnt = 0;
  spPose vicon_pose;
  spPose vicon_prev_pose;
  spTimestamp timestamp;
  spTimestamp prev_timestamp;
  spLinVel linvel;
  spRotVel rotvel;
bool flag0 = true;
  while(spworld.ShouldRun()) {
    // Get Car's Pose
#ifdef SIM_CALIB
    vicon_prev_pose = vicon_pose;
    prev_timestamp = timestamp;

//    vicon_pose = car.GetPose();
    vicon_pose = posys_;
    timestamp = spGeneralTools::Tick();

    spPose diff = vicon_prev_pose.inverse()*vicon_pose;
    linvel = (vicon_pose.translation()-vicon_prev_pose.translation())/0.1;
    Eigen::AngleAxisd angleaxis(diff.rotation());
    Eigen::Vector3d rotvec(angleaxis.angle()*angleaxis.axis());
    rotvel = rotvec/(spGeneralTools::TickTock_us(prev_timestamp,timestamp)/1e6);
    if(flag0) {
      linvel = spLinVel::Zero();
      rotvel = spRotVel::Zero();
      flag0 = false;
    }
#endif
    // Ger Car's wheel speeds
    spWheelSpeedVec wheel_speeds;
    for(int ii=0; ii<4; ii++) {
      wheel_speeds[ii] = car.GetWheel(ii)->GetWheelSpeed();
    }
    // create car state from pose and wheel odometry
    spState current_state/*(car.GetState())*/;
//    current_state.substate_vec.clear();
    current_state.pose = vicon_pose;
    //current_state.wheel_speeds = wheel_speeds;
    //current_state.front_steering = 0.5*(car.GetWheel(0)->GetSteeringServoCurrentAngle()+car.GetWheel(3)->GetSteeringServoCurrentAngle());
    current_state.front_steering = steering_signal;
    current_state.linvel = linvel;// car.GetState().linvel;
    current_state.rotvel = rotvel;//car.GetState().rotvel;
    current_state.time_stamp = timestamp;//spGeneralTools::Tick();
    current_state.current_controls.first = 0.8*steering_signal;
    current_state.current_controls.second = 1.6*throttle_signal;
    candidate_window.PushBackState(current_state);

//    std::cout << "angle is " << car.GetWheel(0)->GetSteeringServoCurrentAngle() << std::endl;

//    spVehicleConstructionInfo params;
//    if(priority_queue.GetParameters(params)) {
//      candidate_window.SetParams(params);
//    }
    spVehicleConstructionInfo params;
    if(entropytable.GetParameters(params)) {
      candidate_window.SetParams(params);
    }



//    std::cout << "**********************" << std::endl;
//    std::cout << "cars is\t" << car.GetState().rotvel.transpose() << std::endl;
//    std::cout << "mine is\t" << rotvel.transpose() << std::endl;
    commandMSG.set_steering_angle(-steering_signal);
    commandMSG.set_throttle_percent(-throttle_signal);
    ninja_car.UpdateCarCommand(commandMSG);

#ifdef SIM_CALIB
   car.SetPose(vicon_pose);
   car.SetFrontSteeringAngle(steering_signal);

   if(cnt == 40) {
       cnt=0;
       car2.SetState(current_state);
   }else {
       cnt++;
    }
   car2.SetFrontSteeringAngle(current_state.current_controls.first);
   car2.SetEngineMaxVel(current_state.current_controls.second);
   //double tdif=spGeneralTools::TickTock_ms(prev_timestamp,timestamp)/1e3;
   //std::cout << "val " << tdif << std::endl;
    //spworld.objects_.StepPhySimulation(0.1);
    // car.SetFrontSteeringAngle(current_state.current_controls.first);
    //car.SetEngineMaxVel(current_state.current_controls.second);
    // step forward the simulated car
    spTimestamp t0 = spGeneralTools::Tick();
    spworld.objects_.StepPhySimulation(0.1);
    double tt = spGeneralTools::Tock_ms(t0);

    spGeneralTools::Delay_ms(100-tt);
    //spGeneralTools::Delay_ms(100);
    spworld.gui_.Iterate(spworld.objects_);

#endif
  }
  std::cout << "Done ... !" << std::endl;
  return 0;
}
