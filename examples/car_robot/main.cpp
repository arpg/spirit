/// Sample Ccommandline Argument
// ./ninja_gui --groundmeshfile=/Users/saghli/code/datasets/meshes/lab.ply --paramfile=/Users/saghli/code/spirit/parameter_files/gui_params.csv

//#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <spirit/spirit.h>
#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>

//DEFINE_string(groundmeshfile, "", "Specify File Path");
//DEFINE_string(paramfile, "", "Specify File Path");
//DEFINE_int32(verbosity, 2,
//             "verbositylevel of node (the lower the less verbose)");

void GamepadCallback(hal::GamepadMsg& _msg) {
  std::cout << "-> "
            << _msg.axes().data(0) << ", "
            << _msg.axes().data(1) << ", "
            << _msg.axes().data(2) << ", "
            << _msg.axes().data(3) << ", "
            << _msg.axes().data(4) << ", "
            << _msg.axes().data(5) << ", "
            << _msg.axes().data(6) << ", "
            << _msg.axes().data(7) << ", "
            << _msg.axes().data(8) << ", "
            << _msg.axes().data(9) << ", "
            << _msg.axes().data(10) << ", "
            << _msg.axes().data(11) << ", "
            << _msg.axes().data(12) << ", "
            << _msg.axes().data(13) << ", "
            << _msg.axes().data(14) << " -  "
            << _msg.buttons().data(0) << ","
            << _msg.buttons().data(1) << ","
            << _msg.buttons().data(2) << ","
            << _msg.buttons().data(3) << ","
            << _msg.buttons().data(4) << ","
            << _msg.buttons().data(5) << ","
            << _msg.buttons().data(6) << ","
            << _msg.buttons().data(7) << ","
            << _msg.buttons().data(8) << ","
            << _msg.buttons().data(9) << ","
            << _msg.buttons().data(10) << ","
            << _msg.buttons().data(11) << ","
            << std::endl;
}

int main(int argc, char** argv) {
  // Read commandline args
//  google::SetUsageMessage(
//      "this is a massage to show how to use the code or"
//      " info about the program.");
//  google::ParseCommandLineFlags(&argc, &argv, true);
//  google::InitGoogleLogging(argv[0]);

  // Example of how to read a gamepad
//  hal::Gamepad gamepad("gamepad:/");
//  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);
  settings_obj.SetNumThreads(1);

  spirit sp_world(settings_obj);
//  sp_world.ScenarioPlannerTest();
//  sp_world.SenarioCeresTest();
//  sp_world.ScenarioPIDController();
//  sp_world.ScenarioGNTest();

//  sp_world.multithreadtest();
  sp_world.SenarioControllerTest();
//  sp_world.zibil();
//  sp_world.DummyTests();

  while(sp_world.ShouldRun()) {
    sp_world.IterateWorld();
    sp_world.CheckKeyboardAction();
  }
  std::cout << "Done ... !" << std::endl;
  return 0;
}
