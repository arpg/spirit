/// Sample Ccommandline Argument
// ./ninja_gui --groundmeshfile=/Users/saghli/code/datasets/meshes/lab.ply --paramfile=/Users/saghli/code/spirit/parameter_files/gui_params.csv

#include <glog/logging.h>
#include <spirit/Gui.h>
#include <thread>
#include <chrono>
#include <math.h>

DEFINE_string(groundmeshfile, "", "Specify File Path");
DEFINE_string(paramfile, "", "Specify File Path");
DEFINE_int32(verbosity, 2,
             "verbositylevel of node (the lower the less verbose)");

int main(int argc, char** argv) {
  // Read commandline args
  google::SetUsageMessage(
      "this is a massage to show how to use the code or"
      " info about the program.");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Initialize a spirit gui
  SpiritGui ninja_gui;

  // add a axis frame
  Eigen::Vector6d axis_pose;
  axis_pose << 0, 0, 0, 0, 0, 0;
  ninja_gui.axes_.AddObj(axis_pose);

  // add a ground mesh to gui
  Eigen::Vector6d mesh_pose;
  mesh_pose << 0, 0, 0, 0, 0, 0;
  ninja_gui.groundmesh_.SetMeshFilePath(FLAGS_groundmeshfile.c_str());
  ninja_gui.groundmesh_.AddObj(mesh_pose);

  // Add a car
  ninja_gui.cars_.InitCarParams(FLAGS_paramfile.c_str());
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
