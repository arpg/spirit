#include <glog/logging.h>
#include <spirit/Gui.h>
#include <chrono>

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
//  SpiritGui ninja_gui;
//  ninja_gui.Init();

  // add a ground mesh to gui
//  ninja_gui.AddGroundMesh(FLAGS_file.c_str());


//  while (ninja_gui.Render()) {
  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
  }

  return 0;
}
