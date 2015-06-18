#include <spirit/Gui.h>

SpiritGui::SpiritGui()
    : axes_(glgraph_),
      waypoints_(glgraph_),
      groundmesh_(glgraph_),
      cars_(groundmesh_.GetCollisionShape()),
      glrenderstate_(
          pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                     WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.01,
                                     1000),
          pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisZ)) {
  pangolin::CreateWindowAndBind("Main", WINDOW_WIDTH, WINDOW_HEIGHT);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();
  gllight_ = new SceneGraph::GLLight(10, 10, -100);
  glgraph_.AddChild(gllight_);
  glgrid_ = new SceneGraph::GLGrid(10, 1, true);
  glgraph_.AddChild(glgrid_);
  handler_sg_ = new SceneGraph::HandlerSceneGraph(glgraph_, glrenderstate_,
                                                  pangolin::AxisZ,0.01f);
  view3d_.SetBounds(0.0, 1.0, 0.0, 1.0, -(double)WINDOW_WIDTH / WINDOW_HEIGHT)
      .SetHandler(handler_sg_)
      .SetDrawFunction(
          SceneGraph::ActivateDrawFunctor(glgraph_, glrenderstate_));
  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(view3d_);
}

SpiritGui::~SpiritGui() {
  delete gllight_;
  delete glgrid_;
  delete handler_sg_;
}

bool SpiritGui::Render(void) {
  if (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    return true;
  }
  return false;
}
