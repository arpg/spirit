#include <gflags/gflags.h>
#include <glog/logging.h>

#include <spirit/Gui.h>
#include <thread>
#include <chrono>
#include <math.h>

DEFINE_int32(verbosity, 2,
             "verbositylevel of node (the lower the less verbose)");

static const char* g_usage =
"Examples: \n\n"
" ninja_gui -mesh /home/user/data/environment_mesh.ply -params /home/user/data/car_params.csv" ;

int main(int argc, char *argv[]) {
  // Read commandline args
  if( argc == 1 ) {
    google::SetUsageMessage(g_usage);
    google::ShowUsageWithFlags(argv[0]);
    return -1;
  }

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting NinjaGui.";

  // Initialize a spirit gui
  SpiritGui ninja_gui;

  bool flag = true;
  while (ninja_gui.Render()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (flag) {
      ninja_gui.cars_.UpdateVisualsFromPhysics(0);
//      flag = false;
    }
  }

  return 0;
}
