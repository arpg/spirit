/// Sample Ccommandline Argument
// ./ninja_gui --file=/Users/Sina/rpg/datasets/meshes/lab.ply

#include <glog/logging.h>
#include <spirit/Gui.h>
#include <thread>
#include <chrono>

// Define glog variables
DEFINE_string(file, "", "Specify File Path");
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

  // add a ground mesh to gui
  Eigen::Vector6d mesh_pose;
  mesh_pose << 0, 0, 0, 0, 0, 0;
  ninja_gui.groundmesh_.SetMeshFilePath(FLAGS_file.c_str());
  ninja_gui.groundmesh_.AddObj(mesh_pose);

  // add some axis
  for (int i = 1; i <= 5; i++) {
    Eigen::Vector6d axis_pose;
    axis_pose << i, 1, 1, 0, 0, 0;
    ninja_gui.axes_.AddObj(axis_pose);
  }

  // remove the last axis
  ninja_gui.axes_.DelObj(ninja_gui.axes_.NumOfObjs() - 1);

  // add some waypoints
  for (int i = 1; i <= 5; i++) {
    Eigen::Vector6d waypoint_pose;
    waypoint_pose << 0, i, 1, 0, 0, 0;
    ninja_gui.waypoints_.AddObj(waypoint_pose);
  }

  // remove the last waypoint
  ninja_gui.waypoints_.DelObj(ninja_gui.waypoints_.NumOfObjs() - 1);

  // Add a car
  ninja_gui.cars_.InitCarParams(
      "/Users/Sina/rpg/spirit/parameter_files/gui_params.csv");
  ninja_gui.cars_.InitializeMap(ninja_gui.groundmesh_.GetCollisionShape());
  for (int i = 1; i <= 5; i++) {
    Eigen::Vector6d car_pose;
    car_pose << -1, 0, i, 0, 0, 0;
    ninja_gui.cars_.AddObj(car_pose);
    ninja_gui.cars_.SetCarVisibility(i - 1, true);
  }
  ninja_gui.cars_.DelObj(ninja_gui.cars_.NumOfObjs() - 1);

  while (ninja_gui.Render()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 50));
    ninja_gui.cars_.UpdateGuiFromPhysics(0);
  }

  return 0;
}
