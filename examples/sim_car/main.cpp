#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <thread>
//#include <signal.h>
#include <spirit/spirit.h>
#include <HAL/Posys/PosysDevice.h>
#include <iostream>
#include <fstream>
#include <cmath>


//hal::CarCommandMsg commandMSG;

//void GamepadCallback(hal::GamepadMsg& _msg) {
//  commandMSG.set_steering_angle(_msg.axes().data(0));
//  commandMSG.set_throttle_percent(_msg.axes().data(5)*20);
//}

double V(double th, double x, double y, double v){
  return 11.7355150858*th*th + 9.57145993336*x*x + 6.11955646782*y*y + 8.35433503106*v*v - 7.53735437299*th*x + 6.06073796355*th*y - 7.04427801438*th*v + 3.41207480872*x*y - 3.76334532872*v*x + 2.57409404743*v*y;
}

double dV(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double th_t = th + (1.2667*p - 1.6268);
  double sin_th = (-0.1021*th_t*th_t*th_t+0.0409*th_t*th_t+0.9522*th_t-0.0121);
  double cos_th = (-0.0951*th_t*th_t*th_t-0.5044*th_t*th_t+0.0733*th_t+1.0073);
  double v_t  = v  + (1.43);
  double r = (2*11.7355150858*th - 7.53735437299*x + 6.06073796355*y - 7.04427801438*v)*(u_2-(1.2667)*(1+u_3)) +
    (2*9.57145993336*x - 7.53735437299*th + 3.41207480872*y - 3.76334532872*v)*((v_t*-sin_th)-(-0.5435*p*2 + 1.5204)*(1+u_3)) +
    (2*6.11955646782*y + 6.06073796355*th + 3.41207480872*x + 2.57409404743*v)*((v_t*cos_th)-(0.6758*p*2 + 0.2184)*(1+u_3)) +
    (2*8.35433503106*v - 7.04427801438*th - 3.76334532872*x + 2.57409404743*y)*(u_1);
  return r;
}

int main(int argc, char** argv) {
  // connect to a gamepad
//  hal::Gamepad gamepad("gamepad:/");
//  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Initialize commands
//  commandMSG.set_steering_angle(0);
//  commandMSG.set_throttle_percent(0);

  // Create world setting object
  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  // create the world object
  spirit spworld(settings_obj);

  // create a car with default values at car_param object
  spObjectHandle car_handle = spworld.objects_.CreateVehicle(spworld.car_param);
  spworld.gui_.AddObject(spworld.objects_.GetObject(car_handle));
  spAWSDCar& car = (spAWSDCar&) spworld.objects_.GetObject(car_handle);

  // create a flat ground with a box object
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = spworld.objects_.CreateBox(gnd_pose_,spBoxSize(50,50,1),0,spColor(0,1,0));
  spworld.gui_.AddObject(spworld.objects_.GetObject(gnd_handle));

  // set friction coefficent of ground
  ((spBox&)spworld.objects_.GetObject(gnd_handle)).SetFriction(1);


  double x0 = -1;
  double y0 = -1;
  double th0 = -1.6268;
  double v0 = 1.43;
  double p0 = 0;

  double torque0 = 0; // range: -50, 50
  double turn0 = 1.2667; //SP_PI_QUART; // range: -pi/4, pi/4
  double dp0 = 0;

  std::ifstream in_file;
  in_file.open ("temp_files/sim_input.txt");
  in_file >> th0 >> x0 >> y0 >> v0 >> torque0 >> turn0;
  in_file.close();

  double v0_x = -v0*sin(th0);
  double v0_y = v0*cos(th0);

  std::cout << "state: " << th0 << ", " << x0 << ", " << y0 << ", " << v0_x << ", " << v0_y << ", " << torque0 << ", " << turn0 << std::endl;
  
// int first_time = 1;
while(1){
  spState initState;
  initState.pose = spPose::Identity();
  initState.pose.translate(spTranslation(x0,y0,0.06));
  Eigen::AngleAxisd initRot(th0,Eigen::Vector3d::UnitZ());
  initState.pose.rotate(initRot);
  initState.linvel = spVector3(v0_x, v0_y, 0);

  car.SetState(initState);

  int i = 0;

  double p = p0;
  double u_1_prev = torque0;
  double u_2_prev = turn0;
  double u_3_prev = dp0;

  double tau = 0.01;
  while(p < 1) {
//    car.SetEngineMaxVel(commandMSG.throttle_percent());
//    car.SetFrontSteeringAngle(commandMSG.steering_angle());

    // set some constant values for engine velocity and front steering angle
    // Input
    
    Eigen::Matrix3d rotmat = car.GetPose().rotation();
    double x_t = car.GetState().pose.translation()[0];
    double y_t = car.GetState().pose.translation()[1];
    double th_t = std::atan2(rotmat(1,0),rotmat(0,0));
    double v_t = car.GetState().linvel.norm();
    p += (1+u_3_prev)*tau;

    std::cout << "state: " << th_t << ", " << x_t << ", " << y_t << ", " << v_t << ", " << p << std::endl;

    double th = th_t - (1.2667*p - 1.6268);
    double x  = x_t  - (-0.5435*p*p + 1.5204*p - 1.0035);
    double y  = y_t  - (0.6758*p*p + 0.2184*p - 1.0164);
    double v  = v_t  - (1.43);

    double L = V(th, x, y, v);
    std::cout << "L: " << L << std::endl;

    double sin_th = (-0.1021*th_t*th_t*th_t+0.0409*th_t*th_t+0.9522*th_t-0.0121);
    double cos_th = (-0.0951*th_t*th_t*th_t-0.5044*th_t*th_t+0.0733*th_t+1.0073);

    double b_1 = (2*8.35433503106*v - 7.04427801438*th - 3.76334532872*x + 2.57409404743*y);
    double b_2 = (2*11.7355150858*th - 7.53735437299*x + 6.06073796355*y - 7.04427801438*v);
    double b_3 = (2*11.7355150858*th - 7.53735437299*x + 6.06073796355*y - 7.04427801438*v)*(0-(1.2667)) +
    (2*9.57145993336*x - 7.53735437299*th + 3.41207480872*y - 3.76334532872*v)*(0-(-0.5435*p*2 + 1.5204)) +
    (2*6.11955646782*y + 6.06073796355*th + 3.41207480872*x + 2.57409404743*v)*(0-(0.6758*p*2 + 0.2184)) +
    (2*8.35433503106*v - 7.04427801438*th - 3.76334532872*x + 2.57409404743*y)*(0);
    double a = (2*11.7355150858*th - 7.53735437299*x + 6.06073796355*y - 7.04427801438*v)*(0-(1.2667)*(1+0)) +
    (2*9.57145993336*x - 7.53735437299*th + 3.41207480872*y - 3.76334532872*v)*((v_t*-sin_th)-(-0.5435*p*2 + 1.5204)*(1+0)) +
    (2*6.11955646782*y + 6.06073796355*th + 3.41207480872*x + 2.57409404743*v)*((v_t*cos_th)-(0.6758*p*2 + 0.2184)*(1+0)) +
    (2*8.35433503106*v - 7.04427801438*th - 3.76334532872*x + 2.57409404743*y)*(0);

    double beta = b_1*b_1 + b_2*b_2 + b_3*b_3;

    double beta_low = 0.001;
    double u_1 = 0;
    double u_2 = 0;
    double u_3 = 0;

    double u1_max = 4;
    double u1_min = -4;
    double u2_max = 2.45;
    double u2_min = -2.45;
    double u3_max = 9;
    double u3_min = -0.9;
    

    double dL = 0;
    if (L < 0.1){ // use default u if L is small
      u_1 = 0;
      u_2 = 1.2667;
      u_3 = 0;
      dL = dV(u_1, u_2, u_3, th, x, y, v, p);
    }else if (L < 0.2 && u_1_prev == 0 && u_2_prev == -1.2667 && u_3_prev == 0){ // continue using default u
      u_1 = u_1_prev;
      u_2 = u_2_prev;
      u_3 = u_3_prev;
      dL = dV(u_1, u_2, u_3, th, x, y, v, p);
    }else{
      double phi = 0;
      double dec_rate = 0;
      if (beta > beta_low){ // Sontage formula
        if (beta > 5){
          dec_rate = 0.1*sqrt(a*a+beta*beta*beta);
        } else if (beta > 0.5){
          dec_rate = 0.1*sqrt(a*a+beta*beta);
        }else{
          dec_rate = 0.01*sqrt(a*a+beta);
        }
        phi = (dec_rate+a)/beta;
        u_1 = -b_1*phi;
        u_2 = -b_2*phi;
        u_3 = -b_3*phi;
        u_1 = (u_1>u1_max)?u1_max:((u_1<u1_min)?u1_min:u_1);
        u_2 = (u_2>u2_max)?u2_max:((u_2<u2_min)?u2_min:u_2);
        u_3 = (u_3>u3_max)?u3_max:((u_3<u3_min)?u3_min:u_3);
      }
      dL = dV(u_1, u_2, u_3, th, x, y, v, p);

      double thresh = -0.1*L;
      double coef = 1.1;
      while (dL > thresh){ // increase decrease rate size if L is not decreasing
        if(phi == 0 || beta < beta_low)
          break;
        dec_rate = coef*dec_rate;
        phi = (dec_rate + a)/beta;
        double u_1t = -b_1*phi; double u_2t = -b_2*phi; double u_3t = -b_3*phi;
        u_1t = (u_1t>u1_max)?u1_max:((u_1t<u1_min)?u1_min:u_1t);
        u_2t = (u_2t>u2_max)?u2_max:((u_2t<u2_min)?u2_min:u_2t);
        u_3t = (u_3t>u3_max)?u3_max:((u_3t<u3_min)?u3_min:u_3t);
        // std::cout << ";" << dec_rate << ", " << phi << ":" << u_1t << ", " << u_2t << ", " << u_3t << std::endl;
        double dL_t = dV(u_1t, u_2t, u_3t, th, x, y, v, p);
        if (dL_t > dL){
          std::cout << "error: dL increases... V:" << L << ", Beta:" << beta << std::endl;
          // std::cout << "\t " << u_1 << ", " << u_2 << ", " << u_3;
          // std::cout << " : " << u_1t<< ", " << u_2t<< ", " << u_3t << std::endl;
          // std::cout << "\t " << dL << " : " << dL_t << " : " << (a + u_1t*b_1 + u_2t*b_2 + u_3t*b_3) << std::endl; 
          // std::cout << "\t" << b_1 << ", " << b_2 << ", " << b_3 << ", " << phi << std::endl;
          break;
        }
        dL = dL_t; u_1 = u_1t; u_2 = u_2t; u_3 = u_3t;
        if ((u_1 == u1_min || u_1 == u1_max) && (u_2 == u2_min || u_2 == u2_max)  && (u_3 == u3_min || u_3 == u3_max)){
          break;
        }
        // std::cout << ":" << u_1 << ", " << u_2 << ", " << u_3 << std::endl;
      }
      if (dL > thresh)
        std::cout << "dL :" << dL << std::endl;
    }


    u_1_prev = u_1;
    u_2_prev = u_2;
    u_3_prev = u_3;
    // calculate output

    double torque = (u_1-0.3124)/13908;
    // double turn = u_2/-3.5266;
    // if (turn > 0.5){
    //   double torque_adj = 0.005*turn*turn;
    //   std::cout << "torque adj: " << torque_adj << std::endl;
    //   torque += ((torque>0)?1:-1)*torque_adj;
    // }
    // torque = (torque > 0.001)?0.001:((torque<-0.001)?-0.001: torque);
    torque += 0.0002;

    // u_2 = -0.067731*turn -0.026598*v -2.9128*(turn*v) -0.077826*(turn*turn) + 0.011698*(v*v);
    double quad_a = -0.077826;
    double quad_b = -0.067731 -2.9128*v_t;
    double quad_c = -0.026598*v_t + 0.011698*(v_t*v_t) - (u_2);
    double quad_root1 = (-quad_b + sqrt(quad_b*quad_b - 4*quad_a*quad_c))/(2*quad_a);
    double quad_root2 = (-quad_b - sqrt(quad_b*quad_b - 4*quad_a*quad_c))/(2*quad_a);
    u2_max = u2_max/3.5266;
    u2_min = u2_min/3.5266;
    quad_root1 = (quad_root1>u2_max)?u2_max:((quad_root1<u2_min)?u2_min:quad_root1);
    quad_root2 = (quad_root2>u2_max)?u2_max:((quad_root2<u2_min)?u2_min:quad_root2);
    double turn = u_2/-3.5266;
    if (quad_root1 < u2_max && quad_root1 > u2_min && abs(quad_root1 - turn) < abs(quad_root2 - turn)){
      turn = quad_root1;
    }else if (quad_root2 < u2_max && quad_root2 > u2_min && abs(quad_root2 - turn) < abs(quad_root1 - turn)){
      turn = quad_root2;
    }
    std::cout << "turn: " << quad_root1 << " , " << quad_root2 << " , " << (u_2/-3.5266) << std::endl;
    // double turn = quad_root1;
    // std::cout << "turn: " << quad_root1 << " , " << quad_root2 << " , " << (u_2/-3.5266) << std::endl;
    // if(abs(quad_root1 - (u_2/-3.5266)) > abs(quad_root2 - (u_2/-3.5266))){
    //   turn = quad_root2;
    // }
    // if (abs(turn - (u_2/-3.5266)) > 0.5 ){
    //   turn = u_2/-3.5266;
    // }
    // if (u_1*u_1 < 0.00005*0.00005){
    //   turn *= 2;
    // }
    // double turn_adj = 0.01*u_1*u_1;
    // turn += ((turn>0)?1:-1)*turn_adj;
    // if (turn > 0.8) turn = 0.8;
    // if (turn < -0.8) turn = -0.8;
    car.SetEngineMaxVel(100);
    car.SetEngineTorque(torque);
    car.SetFrontSteeringAngle(turn);
    
    std::cout << "control: " << torque << ", " << turn << std::endl;

    spworld.objects_.StepPhySimulation(tau);

    spworld.gui_.Iterate(spworld.objects_);
    std::this_thread::sleep_for(std::chrono::nanoseconds((int)(tau*1000000*10)));
    i++;
  }
}
  return 0;
}
