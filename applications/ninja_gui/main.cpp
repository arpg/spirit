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

  // add a axis frame
  Eigen::Vector6d axis_pose;
  axis_pose << 0, 0, 0, 0, 0, 0;
  ninja_gui.axes_.AddObj(axis_pose);

  // add a ground mesh to gui
  Eigen::Vector6d mesh_pose;
  mesh_pose << 0, 0, 0, 0, 0, 0;

  LOG(INFO) << "Loading mesh file.";
  ninja_gui.groundmesh_.SetMeshFilePath();
  ninja_gui.groundmesh_.AddObj(mesh_pose);

  // Add a car
  LOG(INFO) << "Loading car parameters.";
  ninja_gui.cars_.InitCarParams();
  ninja_gui.cars_.InitializeMap(ninja_gui.groundmesh_.GetCollisionShape());
  Eigen::Vector6d car_pose;
  car_pose << -3.5, 0.9, -1, 0, 0, -0.3;
  ninja_gui.cars_.AddObj(car_pose);
  ninja_gui.cars_.SetCarVisibility(0, true);
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
