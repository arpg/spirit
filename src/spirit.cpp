#include <spirit/spirit.h>

spirit::spirit(const spSettings& user_settings){
  user_settings_ = user_settings;
  // create gui object if requested
  if (user_settings_.GetGuiType() != spGuiType::GUI_NONE) {
    gui_.Create(user_settings_.GetGuiType());
  }
  // add physics world
  objects_ = std::make_shared<Objects>(spPhyEngineType::PHY_BULLET);

  // Initialize default car parameters
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
  car_param.wheel_rollingfriction = 0.1;
  car_param.wheel_friction = 0.5;
  car_param.wheel_width = 0.04;
  car_param.wheel_radius = 0.05;//0.057;
  car_param.susp_damping = 0;
  car_param.susp_stiffness = 10;
  car_param.susp_preloading_spacer = 0.1;
  car_param.susp_upper_limit = 0.013;
  car_param.susp_lower_limit = -0.028;
  car_param.wheel_mass = 0.1;
  car_param.chassis_mass = 5;
  car_param.engine_torque = 100;
  car_param.steering_servo_lower_limit = -SP_PI / 4;
  car_param.steering_servo_upper_limit = SP_PI / 4;
  car_param.steering_servo_max_velocity = 100;
  car_param.steering_servo_torque = 100;
}

spirit::~spirit() {}

bool spirit::ShouldRun() {
  if (gui_.ShouldQuit()) {
    return false;
  } else {
    return true;
  }
}

void spirit::CheckKeyboardAction() { gui_.CheckKeyboardAction(); }

void spirit::ScenarioWorldBoxFall() {

  spPose pose(spPose::Identity());
  pose.translate(spTranslation(0, 0, 2));
//  Eigen::AngleAxisd ang(M_PI / 5, Eigen::Vector3d::UnitY());
//  pose.rotate(ang);

  obj_box_index = objects_->CreateBox(pose, spBoxSize(1, 1, 1), 1, spColor(1, 0, 0));
  gui_.AddObject(objects_->GetObject(obj_box_index));

  // create and add a ground as a box to objects_ vector
  obj_gnd_index = objects_->CreateBox(spPose::Identity(), spBoxSize(10, 10, 1),0, spColor(0, 1, 0));
  gui_.AddObject(objects_->GetObject(obj_gnd_index));
}


