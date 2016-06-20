/// Sample Ccommandline Argument
// ./ninja_gui --groundmeshfile=/Users/saghli/code/datasets/meshes/lab.ply --paramfile=/Users/saghli/code/spirit/parameter_files/gui_params.csv

#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <spirit/spirit.h>

DEFINE_string(groundmeshfile, "", "Specify File Path");
DEFINE_string(paramfile, "", "Specify File Path");
DEFINE_int32(verbosity, 2,
             "verbositylevel of node (the lower the less verbose)");

int main(int argc, char** argv) {
  // Read commandline args
//  google::SetUsageMessage(
//      "this is a massage to show how to use the code or"
//      " info about the program.");
//  google::ParseCommandLineFlags(&argc, &argv, true);
//  google::InitGoogleLogging(argv[0]);

  spSettings settings_obj;
  settings_obj.SetGuiType(spGuiType::GUI_PANGOSCENEGRAPH);
  settings_obj.SetPhysicsEngineType(spPhyEngineType::PHY_BULLET);

  spirit sp_world(settings_obj);
  sp_world.Create();
//  sp_world.ScenarioWorldBoxFall();
  sp_world.ScenarioWorldCarFall();
  while(sp_world.ShouldRun()) {
    sp_world.IterateWorld();
    sp_world.CheckKeyboardAction();
  }
  std::cout << "Done ... !" << std::endl;
  return 0;
}
