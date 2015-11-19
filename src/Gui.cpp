#include <spirit/Gui.h>

//////////////////////////////////////////////////////////////////////////
SpiritGui::SpiritGui() :
  axes_(glgraph_),
  waypoints_(glgraph_),
  groundmesh_(glgraph_),
  cars_(glgraph_),
  glrenderstate_(
      pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                 WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.01,
                                 1000),
      pangolin::ModelViewLookAt(-5, 0, -3, 0, 0, 0, pangolin::AxisNegZ))
{
  pangolin::CreateWindowAndBind("Main", WINDOW_WIDTH, WINDOW_HEIGHT);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();

  handler_sg_ = new SceneGraph::HandlerSceneGraph(glgraph_, glrenderstate_,
                                                  pangolin::AxisNegZ,0.01f);
  view3d_.SetBounds(0.0, 1.0, 0.0, 1.0, -(double)WINDOW_WIDTH / WINDOW_HEIGHT)
      .SetHandler(handler_sg_)
      .SetDrawFunction(
          SceneGraph::ActivateDrawFunctor(glgraph_, glrenderstate_));

  // Set up keybindings.
  setup_keybindings();

  groundmesh_.ImportResources();

  Init();

}

//////////////////////////////////////////////////////////////////////////
void SpiritGui::Init() {
  gllight_ = new SceneGraph::GLLight(0, 0, -0.1);
  glgraph_.AddChild(gllight_);
  glgrid_ = new SceneGraph::GLGrid(10, 1, true);
  glgraph_.AddChild(glgrid_);

  // Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(view3d_);

  /// Build up the necessary components.
  // Add an axis frame
  Eigen::Vector6d axis_pose;
  axis_pose << 0, 0, 0, 0, 0, 0;
  axes_.AddObj(axis_pose);

  // Add a ground mesh to gui
  Eigen::Vector6d mesh_pose;
  mesh_pose << 0, 0, 0, 0, 0, 0;

  LOG(INFO) << "Loading mesh file.";
  groundmesh_.SetMeshFilePath();
  groundmesh_.AddObj(mesh_pose);

  // Add a car
  LOG(INFO) << "Loading car parameters.";
  cars_.InitCarParams();
  cars_.InitializeMap(groundmesh_.GetCollisionShape());
  Eigen::Vector6d car_pose;
  car_pose << -3.5, 0.9, -1, 0, 0, -0.3;
  cars_.AddObj(car_pose);
  cars_.SetCarVisibility(0, true);
}

//////////////////////////////////////////////////////////////////////////
void SpiritGui::Clear() {
  cars_.Clear();
  groundmesh_.Clear();
  axes_.Clear();
  waypoints_.Clear();
  glgraph_.Clear();
  delete gllight_;
  delete glgrid_;
}

//////////////////////////////////////////////////////////////////////////
SpiritGui::~SpiritGui() {
  delete gllight_;
  delete glgrid_;
  delete handler_sg_;
}

//////////////////////////////////////////////////////////////////////////
bool SpiritGui::Render() {
  if (!pangolin::ShouldQuit() ) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    return true;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////
void SpiritGui::StartThreads()
{

}

//////////////////////////////////////////////////////////////////////////
void SpiritGui::setup_keybindings() {
  pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r', [&] { this->Clear(); this->Init(); } );
}
