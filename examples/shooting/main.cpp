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
bool flag_auto = false;

spPose vicon_pose;
spPose vicon_prev_pose;
spLinVel ninja_linvel;
spRotVel ninja_rotvel;
spTimestamp vicon_t0;
double vicon_time_elapsed = 0;
bool init_state_flg = true;

void vicon_poseHandler(hal::PoseMsg& PoseData) {
//std::cout << "p0se data" << std::endl;
    vicon_time_elapsed = spGeneralTools::Tock_ms(vicon_t0)/1000.0;
    vicon_t0 = spGeneralTools::Tick();
    vicon_pose = spPose::Identity();
    vicon_pose.translate(spTranslation(PoseData.pose().data(0),PoseData.pose().data(1),0.07/*PoseData.pose().data(2)*/));
    spRotation rot(PoseData.pose().data(6),PoseData.pose().data(3),PoseData.pose().data(4),PoseData.pose().data(5));
    Eigen::AngleAxisd tracker_rot(-SP_PI_HALF,Eigen::Vector3d::UnitZ());
    vicon_pose.rotate(rot);
    vicon_pose.rotate(tracker_rot);


    if(init_state_flg) {
      ninja_linvel = spLinVel::Zero();
      ninja_rotvel = spRotVel::Zero();
      init_state_flg = false;
    } else {
        spPose diff = vicon_prev_pose.inverse()*vicon_pose;
        ninja_linvel = (vicon_pose.translation()-vicon_prev_pose.translation())/vicon_time_elapsed;
	ninja_linvel[2] = 0;
        Eigen::AngleAxisd angleaxis(diff.rotation());
        Eigen::Vector3d rotvec(angleaxis.angle()*angleaxis.axis());
        ninja_rotvel = rotvec/vicon_time_elapsed;
	ninja_rotvel[0] = 0;
	ninja_rotvel[1] = 0;
    }
   vicon_prev_pose = vicon_pose;
}

void GamepadCallback(hal::GamepadMsg& _msg) {
  gamepad_steering = -_msg.axes().data(0);
  gamepad_throttle = _msg.axes().data(4)*40;
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

int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]//");

  //ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  //////////////////////////////
  hal::Posys vicon("vicon://tracker:[ninja]");
  vicon.RegisterPosysDataCallback(&vicon_poseHandler);
  /////////////////////////////
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);
  spirit spworld(settings_obj);

  std::vector<spObjectHandle> cars;
#define num_cars 16
  double disc_step_size = 0.1;
  double simulation_length = 0.6;

  spworld.car_param.pose.translate(spTranslation(1.5,0,0));
  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& estimation_car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);

  // create a flat ground
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);

  /////////////////////////////////

  std::vector<std::shared_ptr<spStateSeries>> sim_traj;
  spState current_state = estimation_car.GetState();
  std::shared_ptr<spState> state_ptr = std::make_shared<spState>(current_state);
  std::vector<std::shared_ptr<CarSimFunctor>> sims;
  std::vector<std::shared_ptr<double>> costs;
  std::shared_ptr<double> tire_friction = std::make_shared<double>(0.5);
  //std::vector<std::shared_ptr<spState>> state_ptr;

  // create search pattern
  std::vector<spCtrlPts2ord_2dof> cntrl_vars_vec(num_cars);
//  for(int jj=0; jj<=2; jj++) {
  int jj=0;
  int speed = 110;
    for(int ii=0; ii<num_cars; ii++) {
      cntrl_vars_vec[(9*jj)+ii].col(0) = Eigen::Vector2d(spworld.car_param.steering_servo_lower_limit+ii*(spworld.car_param.steering_servo_upper_limit-spworld.car_param.steering_servo_lower_limit)/(num_cars-1),speed);
      cntrl_vars_vec[(9*jj)+ii].col(1) = Eigen::Vector2d(0.1,speed);//Eigen::Vector2d(spworld.car_param.steering_servo_lower_limit+ii*(spworld.car_param.steering_servo_upper_limit-spworld.car_param.steering_servo_lower_limit)/(num_cars-1),speed);
      cntrl_vars_vec[(9*jj)+ii].col(2) = Eigen::Vector2d(0.1,speed);//Eigen::Vector2d(spworld.car_param.steering_servo_lower_limit+ii*(spworld.car_param.steering_servo_upper_limit-spworld.car_param.steering_servo_lower_limit)/(num_cars-1),speed);
    }
//  }

  // create cars.
  for(int ii = 0; ii < num_cars; ii++) {
    //state_ptr.push_back(std::make_shared<spState>(current_state));
    sims.push_back(std::make_shared<CarSimFunctor>(spworld.car_param,current_state));
    costs.push_back(std::make_shared<double>(0));
  }

  while(1){
    spTimestamp t0 = spGeneralTools::Tick();
    sim_traj.clear();
    costs.clear();
    for(int ii = 0; ii < num_cars; ii++) {
      costs.push_back(std::make_shared<double>(0));
    }

    // Update current state
    //*state_ptr = estimation_car.GetState();
state_ptr.reset(new spState(current_state));
            state_ptr->pose = vicon_pose;
	    state_ptr->linvel = ninja_linvel;
	    state_ptr->rotvel = ninja_rotvel;

            if(state_ptr->pose.translation()[0]>0) {
		tire_friction.reset(new double(0.5));
	    } else {
                tire_friction.reset(new double(0.5));
	    }

 std::cout << std::fixed << std::setprecision(3) << state_ptr->pose.translation()[0] << ","  << state_ptr->pose.translation()[0] << ";"   <<std::endl;
    
/*
	    state_ptr[ii]->substate_vec[0]->linvel = ninja_linvel;
	    state_ptr[ii]->substate_vec[1]->linvel = ninja_linvel;
	    state_ptr[ii]->substate_vec[2]->linvel = ninja_linvel;
	    state_ptr[ii]->substate_vec[3]->linvel = ninja_linvel;
	
	    state_ptr[ii]->substate_vec[0]->rotvel = ninja_rotvel;
	    state_ptr[ii]->substate_vec[1]->rotvel = ninja_rotvel;
	    state_ptr[ii]->substate_vec[2]->rotvel = ninja_rotvel;
	    state_ptr[ii]->substate_vec[3]->rotvel = ninja_rotvel;
*/

    // Run simulations
    for(int ii = 0; ii < num_cars; ii++) {
      sim_traj.push_back(std::make_shared<spStateSeries>());
      sims[ii]->RunInThread(ii,(int)(simulation_length/disc_step_size), disc_step_size, cntrl_vars_vec[ii], 0, -1, sim_traj[ii], state_ptr,costs[ii],tire_friction);
    }
    for(int ii = 0; ii < num_cars; ii++) {
      sims[ii]->WaitForThreadJoin();
    }


    // Pick lowest cost constrol signal and apply to the vehicle
    int lowest_cost_index = 0;
    double lowest_cost = *(costs[lowest_cost_index]);
    for(int ii=1; ii<num_cars; ii++) {
      if(*(costs[ii])<lowest_cost) {
        lowest_cost = *(costs[ii]);
        lowest_cost_index = ii;
      }
    }

    // calc the processing time
    double time = spGeneralTools::Tock_ms(t0);
    //std::cout << "time is " << time << " , " << gamepad_throttle << std::endl;

    // apply signal to the car
    double steering = (cntrl_vars_vec[lowest_cost_index]).col(0)[0];
	steering *= 1;
    if(flag_auto) {
      commandMSG.set_steering_angle(-steering);
    //commandMSG.set_throttle_percent(-23);
    } else {
      commandMSG.set_steering_angle(gamepad_steering);
    commandMSG.set_throttle_percent(gamepad_throttle);
    }
commandMSG.set_throttle_percent(gamepad_throttle);
    ninja_car.UpdateCarCommand(commandMSG);

//    std::cout << "cost " << lowest_cost << " , " << lowest_cost_index << std::endl;
//    estimation_car.SetFrontSteeringAngle((cntrl_vars_vec[lowest_cost_index]).col(0)[0]);
//    estimation_car.SetEngineMaxVel((cntrl_vars_vec[lowest_cost_index]).col(0)[1]);
    spworld.gui_.Iterate(spworld.objects_);
 //   spworld.objects_.StepPhySimulation(simulation_length);
  //  spworld.gui_.Iterate(spworld.objects_);
//      std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
  return 0;
}
