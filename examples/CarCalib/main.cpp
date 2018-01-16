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
#include <spirit/Types/spTypes.h>

#define SIM_CALIB

double steering_signal = 0;
double throttle_signal = 0;
void GamepadCallback(hal::GamepadMsg& _msg) {
  steering_signal = _msg.axes().data(0)*SP_PI_QUART;
  throttle_signal = -_msg.axes().data(3)*220;
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

  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  spirit spworld(settings_obj);

  class MyVehicleConstInfo : public spVehicleConstructionInfo {
  public:
    MyVehicleConstInfo() {
      vehicle_type = spObjectType::VEHICLE_AWSD;
      pose = spPose::Identity();
      pose.translate(spTranslation(0,0,0.07));
      wheels_anchor.push_back(spTranslation(-0.15, 0.17, -0.003));
      wheels_anchor.push_back(spTranslation(-0.15, -0.17, -0.003));
      wheels_anchor.push_back(spTranslation(0.15, -0.17, -0.003));
      wheels_anchor.push_back(spTranslation(0.15, 0.17, -0.003));
      chassis_size = spBoxSize(0.2, 0.42, 0.05);
      cog = spTranslation(0, 0, 0);
      chassis_friction = 0;
      wheel_rollingfriction = 0.1;
      wheel_friction = 0.3;
      wheel_width = 0.04;
      wheel_radius = 0.05;//0.057;
      susp_damping = 0;
      susp_stiffness = 10;
      susp_preloading_spacer = 0.1;
      susp_upper_limit = 0.013;
      susp_lower_limit = -0.028;
      wheel_mass = 0.1;
      chassis_mass = 5;
      engine_torque = 0.0001;
      steering_servo_lower_limit = -SP_PI / 4;
      steering_servo_upper_limit = SP_PI / 4;
      steering_servo_max_velocity = 2;
      steering_servo_torque = 100;
//      Eigen::VectorXd min_limits(3);
      Eigen::VectorXd min_limits(2);
      min_limits[0] = 0.1;
      min_limits[1] = 0.1;
//      min_limits[2] = 0.1;
      calib_min_limit_vec = min_limits;
//      Eigen::VectorXd max_limits(3);
      Eigen::VectorXd max_limits(2);
      max_limits[0] = 0.4;
      max_limits[1] = 1;
//      max_limits[2] = 0.2;
      calib_max_limit_vec = max_limits;
    }
    MyVehicleConstInfo(const spVehicleConstructionInfo& obj) : spVehicleConstructionInfo(obj) {}
    MyVehicleConstInfo(const MyVehicleConstInfo& obj) : spVehicleConstructionInfo(obj) {}
    std::shared_ptr<spVehicleConstructionInfo> MakeCopy() {
      std::shared_ptr<spVehicleConstructionInfo> cpy = std::make_shared<MyVehicleConstInfo>(*this);
      return cpy;
    }

    Eigen::VectorXd GetParameterVector() const {
//      Eigen::VectorXd vec(3);
      Eigen::VectorXd vec(2);
      vec[0] = (wheels_anchor[0]-wheels_anchor[1])[1];
      vec[1] = wheel_friction;
//      vec[2] = (wheels_anchor[2]-wheels_anchor[1])[0];
      return vec;
    }
    void SetParameterVector(Eigen::VectorXd vec) {
      wheels_anchor[0][1] = vec[0]/2;
      wheels_anchor[1][1] = -vec[0]/2;
      wheels_anchor[2][1] = -vec[0]/2;
      wheels_anchor[3][1] = vec[0]/2;
      wheel_friction = vec[1];
//      wheels_anchor[0][0] = -vec[2]/2;
//      wheels_anchor[1][0] = -vec[2]/2;
//      wheels_anchor[2][0] = vec[2]/2;
//      wheels_anchor[3][0] = vec[2]/2;
    }
  };

  std::shared_ptr<spVehicleConstructionInfo> car_params = std::make_shared<MyVehicleConstInfo>();

#ifdef SIM_CALIB
  spObjectHandle car_handle = spworld.objects_.CreateVehicle(*car_params);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);
  // create a flat ground
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));
  // set ground friction
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);

#endif

  unsigned int window_size = 20;
  unsigned int queue_size = 10;
  double batch_min_entropy = 10;
  // create a candidate_window and a priority queue
//  PriorityQueue priority_queue(queue_size,*car_params);
  // create a new candidate window
  car_params->wheel_friction = 0.3;
  double wheel_base = 0.27;
  car_params->wheels_anchor[0][1] = wheel_base/2;
  car_params->wheels_anchor[1][1] = -wheel_base/2;
  car_params->wheels_anchor[2][1] = -wheel_base/2;
  car_params->wheels_anchor[3][1] = wheel_base/2;
//  double wheel_trach_len = 0.20;
//  car_params->wheels_anchor[0][0] = -wheel_trach_len/2;
//  car_params->wheels_anchor[1][0] = -wheel_trach_len/2;
//  car_params->wheels_anchor[2][0] = wheel_trach_len/2;
//  car_params->wheels_anchor[3][0] = wheel_trach_len/2;

  CandidateWindow candidate_window(window_size,1,car_params/*,&spworld.gui_*/);
  EntropyTable entropytable(car_params,2,5);
//  candidate_window.prqueue_func_ptr_ = std::bind(&PriorityQueue::PushBackCandidateWindow,&priority_queue,std::placeholders::_1);
  candidate_window.prqueue_func_ptr_ = std::bind(&EntropyTable::PushBackCandidateWindow,&entropytable,std::placeholders::_1);
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

    vicon_pose = car.GetPose();
    timestamp = spGeneralTools::Tick();

    spPose diff = vicon_prev_pose.inverse()*vicon_pose;
    linvel = (vicon_pose.translation()-vicon_prev_pose.translation())/0.1;
    Eigen::AngleAxisd angleaxis(diff.rotation());
    Eigen::Vector3d rotvec(angleaxis.angle()*angleaxis.axis());
    rotvel = rotvec/0.1/*(spGeneralTools::TickTock_us(prev_timestamp,timestamp)/1e6)*/;
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
    spState current_state(car.GetState());
//    current_state.substate_vec.clear();

//    current_state.pose = vicon_pose;
//    current_state.wheel_speeds = wheel_speeds;
//    current_state.front_steering = 0.5*(car.GetWheel(0)->GetSteeringServoCurrentAngle()+car.GetWheel(3)->GetSteeringServoCurrentAngle());
//    current_state.linvel = car.GetState().linvel;
//    current_state.rotvel = car.GetState().rotvel;
    current_state.time_stamp = spGeneralTools::Tick();
    current_state.current_controls.first = steering_signal;
    current_state.current_controls.second = throttle_signal;

    candidate_window.PushBackState(current_state);

//    std::cout << "angle is " << car.GetWheel(0)->GetSteeringServoCurrentAngle() << std::endl;

//    spVehicleConstructionInfo params;
//    if(priority_queue.GetParameters(params)) {
//      candidate_window.SetParams(params);
//    }
    std::shared_ptr<spVehicleConstructionInfo> params;
    if(entropytable.GetParameters(params)) {
      candidate_window.SetParams(params);
    }

//    std::cout << "**********************" << std::endl;
//    std::cout << "cars is\t" << car.GetState().rotvel.transpose() << std::endl;
//    std::cout << "mine is\t" << rotvel.transpose() << std::endl;
/*
    if(candidate_window.PushBackState(current_state) == window_size) {
      if(candidate_window.OptimizeParametersInWindow(spworld.car_param,&spworld.gui_)) {
        //      std::cout << "entropy is " << candidate_window.GetEntropy() << std::endl;
        // if candidate window has lower entropy than max entropy of queue then replace with highest one.
        // if there are at least two candidate windows then optimize parameters over whole queue
        if(priority_queue.PushBackCandidateWindow(candidate_window) > 1) {
          priority_queue.OptimizeParametersInQueue(spworld.car_param);
          // update new parameters to test car here
        }
      }
    }
    */
    // apply the controls and wait for next state update
#ifdef SIM_CALIB
    car.SetFrontSteeringAngle(current_state.current_controls.first);
    car.SetEngineMaxVel(current_state.current_controls.second);
    // step forward the simulated car
    spTimestamp t0 = spGeneralTools::Tick();
    spworld.objects_.StepPhySimulation(0.1);
    double tt = spGeneralTools::Tock_ms(t0);

    spGeneralTools::Delay_ms(100-tt);
    spworld.gui_.Iterate(spworld.objects_);

#endif
  }
  std::cout << "Done ... !" << std::endl;
  return 0;
}
