#include <spirit/Gui/spPangolinScenegraphGui.h>

spPangolinScenegraphGui::spPangolinScenegraphGui()
    : a_button_("ui.A_Button", false, false),
      a_double_("ui.A_Double", 3, 0, 5),
      an_int_("ui.An_Int", 2, 0, 5),
      a_double_log_("ui.Log_scale var", 3, 1, 1E4, true),
      a_checkbox_("ui.A_Checkbox", false, true),
      an_int_no_input_("ui.An_Int_No_Input", 2),
      save_window_("ui.Save_Window", false, false) {}

spPangolinScenegraphGui::~spPangolinScenegraphGui() {
  // remove globjects in row
  for (int ii = globjects_.size() - 1; ii >= 0; ii--) {
    glscenegraph_.RemoveChild(globjects_[ii]);
    delete (globjects_[ii]);
  }
}

void spPangolinScenegraphGui::InitGui() {
  if (!spGeneralTools::CheckFileExists(SPIRITGUI_PARAM_FILE)) {
    std::cerr << "Error: Missing Pangolin config file." << std::endl;
  }

  // Load configuration data
  pangolin::ParseVarsFile(SPIRITGUI_PARAM_FILE);

  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind(SPIRITGUI_WINDOW_NAME, 640, 480);

  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glewInit();

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  glrenderstate_ = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                 WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.1,
                                 1000),
      pangolin::ModelViewLookAt(10, 10, 10, 0, 0, 0, pangolin::AxisZ));

  handler_scenegraph_ = new SceneGraph::HandlerSceneGraph(
      glscenegraph_, glrenderstate_, pangolin::AxisZ, 0.01f);

  // Add named OpenGL viewport to window and provide 3D Handler
  pangoview_.SetBounds(0.0, 1.0, 0, 1.0, -(double)WINDOW_WIDTH / WINDOW_HEIGHT)
      .SetHandler(handler_scenegraph_)
      .SetDrawFunction(
          SceneGraph::ActivateDrawFunctor(glscenegraph_, glrenderstate_));

  // Create Globjects
  globjects_.push_back(new SceneGraph::GLGrid(10, 1, false));
  globjects_.push_back(new SceneGraph::GLLight(0, 0, 0));

  // Add already created globjects to glscenegraph_
  for (int ii = 0; ii < globjects_.size(); ii++) {
    glscenegraph_.AddChild(globjects_[ii]);
  }

  pangolin::DisplayBase().AddDisplay(pangoview_);

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_PANEL_WIDTH));

  //////////////////////////////////////////////////
  /// Register Keyboard actions
  pangolin::RegisterKeyPressCallback(
      pangolin::PANGO_CTRL + 'b',
      pangolin::SetVarFunctor<double>("ui.A Double", 3.5));

  // Demonstration of how we can register a keyboard hook to trigger a method
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r',
                                     this->KeyActionMethodSample);
}

bool spPangolinScenegraphGui::ShouldQuit() { return (pangolin::ShouldQuit()); }

void spPangolinScenegraphGui::Refresh() {
  // Clear entire screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Activate efficiently by object
  pangoview_.Activate(glrenderstate_);

  // Swap frames and Process Events
  pangolin::FinishFrame();
}

void spPangolinScenegraphGui::CheckKeyboardAction() {
  if (pangolin::Pushed(a_button_))
    std::cout << "You Pushed a button!" << std::endl;

  // Overloading of Var<T> operators allows us to treat them like
  // their wrapped types, eg:
  if (a_checkbox_) an_int_ = a_double_;

  an_int_no_input_ = an_int_;

  if (pangolin::Pushed(save_window_)) {
    pangolin::SaveWindowOnRender("window");
  }

}

void spPangolinScenegraphGui::KeyActionMethodSample() {
  std::cout
      << "KeyActionMethodSample() method of spPangolinScenegraph is called ..."
      << std::endl;
}
