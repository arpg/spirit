#include <spirit/Gui.h>

SpiritGui::SpiritGui()
    : glrenderstate_(
          pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                     WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.01,
                                     1000),
          pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisZ)),
      axes_(glgraph_),
      waypoints_(glgraph_) {
  pangolin::CreateWindowAndBind("Main", WINDOW_WIDTH, WINDOW_HEIGHT);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();
  gllight_ = new SceneGraph::GLLight(10, 10, -100);
  glgraph_.AddChild(gllight_);
  glgrid_ = new SceneGraph::GLGrid(10, 1, true);
  glgraph_.AddChild(glgrid_);
  handler_sg_ = new SceneGraph::HandlerSceneGraph(glgraph_, glrenderstate_,
                                                  pangolin::AxisZ);
  view3d_.SetBounds(0.0, 1.0, 0.0, 1.0, -(double)WINDOW_WIDTH / WINDOW_HEIGHT)
      .SetHandler(handler_sg_)
      .SetDrawFunction(
          SceneGraph::ActivateDrawFunctor(glgraph_, glrenderstate_));
  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(view3d_);
}

SpiritGui::~SpiritGui() {}

void SpiritGui::AddGroundMesh(std::string file_name) {
  const aiScene* pScene = aiImportFile(
      file_name.c_str(),
      aiProcess_Triangulate | aiProcess_GenSmoothNormals |
          aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes |
          aiProcess_FindInvalidData | aiProcess_FixInfacingNormals);
  glgroundmesh_.Init(pScene);
  glgroundmesh_.SetSelectable(true);
  glgraph_.AddChild(&glgroundmesh_);
}

bool SpiritGui::Render(void) {
  if (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    return true;
  }
  return false;
}
