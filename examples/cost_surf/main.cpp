#include <math.h>
#include <spirit/spirit.h>

int main(int argc, char** argv) {
  CircleMan maneuver(10,3);


  spState state;
  state.pose = spPose::Identity();
  state.pose.translate(spTranslation(10,0,0));
  Eigen::AngleAxisd rot1(0,Eigen::Vector3d::UnitZ());
  state.pose.rotate(rot1);
  state.linvel = spLinVel(0.0,0,0);
  state.rotvel = spRotVel(0,0,0);
  state.wheel_speeds = spWheelSpeedVec(0,0,0,0);

  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);
  spirit spworld(settings_obj);

  int num_residual_blocks_ = 10;
  double simulation_length = 1;
  Eigen::VectorXd res(num_residual_blocks_*12);

  spCtrlPts2ord_2dof controls;
  controls.col(0) = Eigen::Vector2d(0.7,1000);
  controls.col(1) = Eigen::Vector2d(0.7,1000);
  controls.col(2) = Eigen::Vector2d(0.7,1000);

  Eigen::MatrixXd cost_surfs(20,6);
  for(int ii=-10; ii<10; ii++){
    double str = ii*(SP_PI_QUART*0.1);
    controls.col(0) = Eigen::Vector2d(str,1000);
    controls.col(1) = Eigen::Vector2d(str,1000);
    controls.col(2) = Eigen::Vector2d(str,1000);
    CarSimFunctorRK4 sims(spworld.car_param,state);
    std::shared_ptr<spStateSeries> curr_states = std::make_shared<spStateSeries>();
    sims(0, (int)(simulation_length/DISCRETIZATION_STEP_SIZE), DISCRETIZATION_STEP_SIZE, controls, 0, -1, curr_states);

    // Calculate residual
    for(int jj = 0; jj<num_residual_blocks_; jj++) {
      res.block<12,1>(jj*12,0) = -maneuver.GetStateError(*((*curr_states)[jj+1]));
//      std::cout << "x -> " << (*((*curr_states)[jj+1])).pose.translation()[0];
//      std::cout << "   y -> " << (*((*curr_states)[jj+1])).pose.translation()[1] << std::endl;
    }
//    while(1);
    cost_surfs(ii+10,0) = res.norm();
  }
  std::cout << "cost is \n" << cost_surfs << std::endl;
  std::cout << "Done ... !" << std::endl;
  return 0;
}
