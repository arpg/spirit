#include <spirit/Gui.h>

SpiritGui::SpiritGui() {}

SpiritGui::~SpiritGui() {}

void SpiritGui::Init(void) {
  pangolin::CreateWindowAndBind("Main", WINDOW_WIDTH, WINDOW_HEIGHT);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();
  gllight_ = new SceneGraph::GLLight(10, 10, -100);
  glgraph_.AddChild(gllight_);
  glgrid_ = new SceneGraph::GLGrid(10, 1, true);
  glgraph_.AddChild(glgrid_);
  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState glrenderstate(
      pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                 WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.01,
                                 1000),
      pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisZ));
  handler_sg_ = new SceneGraph::HandlerSceneGraph(glgraph_, glrenderstate, pangolin::AxisZ);
  view3d_.SetBounds(0.0, 1.0, 0.0, 1.0, -(double)WINDOW_WIDTH / WINDOW_HEIGHT)
      .SetHandler(handler_sg_)
      .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glgraph_, glrenderstate));
  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(view3d_);
}

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
  if(!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    return true;
  }
  return false;
}

int SpiritGui::AddAxis(Eigen::Vector6d T_w_a) {
  SceneGraph::GLAxis glaxis;
  glaxis.SetPose(T_w_a[0], T_w_a[1], T_w_a[2], T_w_a[3], T_w_a[4], T_w_a[5]);
  glaxis.SetScale(0.5);
  glaxis.SetPerceptable(false);
  axes_vec_.push_back(glaxis);
  glgraph_.AddChild(&axes_vec_.back());
  return axes_vec_.size()-1;
}

int SpiritGui::NumOfAxes(void) {
  return axes_vec_.size();
}

void SpiritGui::DelAxis(int axis_num) {
  axes_vec_.erase(axes_vec_.begin()+axis_num);
}
