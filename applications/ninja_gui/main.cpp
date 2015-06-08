#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/iterator.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <tinyxml2.h>
//#include <Waypointer/WaypointerGui.h>

DEFINE_string(file, "", "Specify File Path");
DEFINE_int32(verbosity, 2,
             "verbositylevel of node (the lower the less verbose)");

int main(int argc, char** argv) {
  google::SetUsageMessage(
      "this is a massage to show how to use the code or"
      " info about the program.");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Create OpenGL window in single line thanks to GLUT
  pangolin::CreateWindowAndBind("Main", 640, 480);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();

  // Scenegraph to hold GLObjects and relative transformations
  SceneGraph::GLSceneGraph glGraph;

  SceneGraph::GLLight light(10, 10, -100);
  glGraph.AddChild(&light);

  SceneGraph::GLGrid grid(10, 1, true);
  glGraph.AddChild(&grid);

  SceneGraph::AxisAlignedBoundingBox bbox;

#ifdef HAVE_ASSIMP
  // Define a mesh object and try to load model
  SceneGraph::GLMesh glMesh;
  try {
    const aiScene* pScene = aiImportFile(
        FLAGS_file.c_str(),
        aiProcess_Triangulate | aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes |
            aiProcess_FindInvalidData | aiProcess_FixInfacingNormals);
    glMesh.Init(pScene);
//    std::string mesh_name("groud_mesh");
//    glMesh.SetObjectName(mesh_name);
    glMesh.SetSelectable(true);
    glGraph.AddChild(&glMesh);
    //    bbox = glMesh.ObjectAndChildrenBounds();
  } catch (std::exception e) {
    std::cerr << "Cannot load mesh." << std::endl;
    std::cerr << e.what() << std::endl;
    exit(-1);
  }
#endif  // HAVE_ASSIMP

  // Define axis object, and set its pose
  SceneGraph::GLAxis glAxis;
  glAxis.SetPose(1, 1, 1, 1, 0, 0);
  glAxis.SetScale(0.5);
  glAxis.SetPerceptable(false);
  glGraph.AddChild(&glAxis);
  //  glMesh.SetPose(0,0,3,0,0,0);
  //  std::cout << "data:\n" << glMesh.GetPose4x4_op()<< std::endl;
  // Optionally clamp waypoint to specific plane
  //  glWaypoint.ClampToPlane(Eigen::Vector4d(0, 0, 1, 0));

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState stacks3d(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.01, 1000),
      pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisZ));

  // We define a new view which will reside within the container.
  pangolin::View view3d;
  SceneGraph::HandlerSceneGraph* my_handler =
      new SceneGraph::HandlerSceneGraph(glGraph, stacks3d, pangolin::AxisZ);

  // We set the views location on screen and add a handler which will
  // let user input update the model_view matrix (stacks3d) and feed through
  // to our scenegraph
  view3d.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
      .SetHandler(my_handler)
      .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));

  // Define movable waypoint object with velocity
  SceneGraph::GLWayPoint glWaypoint;
  glWaypoint.SetPose(0, 0, 1, 0, 0, 0);
  glWaypoint.m_bLocked = false;
  //glWaypoint.ClampToPlane(Eigen::Vector4d(0, 0, 1, 0));
//  glWaypoint.SetSelectable(true);
  //  glWaypoint.ClampToSurface(true,my_handler);
  glGraph.AddChild(&glWaypoint);

  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(view3d);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while (!pangolin::ShouldQuit()) {
    // Clear whole screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Swap frames and Process Events
    pangolin::FinishFrame();
    // Pause for 1/60th of a second.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
  }

  return 0;
}
