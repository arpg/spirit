/// Sample Ccommandline Argument
// ./ninja_gui --groundmeshfile=/Users/saghli/code/datasets/meshes/lab.ply --paramfile=/Users/saghli/code/spirit/parameter_files/gui_params.csv

#include <glog/logging.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <spirit/World.h>

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

  Eigen::Transform<double,3,Eigen::Affine> tr(Eigen::Transform<double,3,Eigen::Affine>::Identity());
  Eigen::AngleAxisd ang(0,Eigen::Vector3d::UnitY());
  tr.rotate(ang);
  std::cout << "transform is:\n" << tr.matrix() << std::endl;
  Eigen::Quaterniond q(tr.rotation());
  std::cout << "Quaternion is:\n" << q.x()<<" , "<< q.y()<<" , "<< q.z()<<" , " << q.w()<<" , "<< std::endl;
  std::cout << "rotation matrix is:\n" << q.toRotationMatrix() << std::endl;

  return 0;

  spirit::Settings settings_obj;
  settings_obj.SetGuiType(spirit::GUI_PANGOSCENEGRAPH);

  spirit::World sp_world(settings_obj);
  sp_world.Create();

  while(sp_world.ShouldRun()) {
    sp_world.IterateGraphics();
    sp_world.CheckKeyboardAction();
  }
  std::cout << "Done ... !" << std::endl;
  return 0;
}
