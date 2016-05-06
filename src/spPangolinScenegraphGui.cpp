#include <spirit/Gui/spPangolinScenegraphGui.h>

spPangolinScenegraphGui::spPangolinScenegraphGui()
    : a_button_("ui.A_Button", false, false),
      a_double_("ui.A_Double", 3, 0, 5),
      an_int_("ui.An_Int", 2, 0, 5),
      a_double_log_("ui.Log_scale var", 3, 1, 1E4, true),
      a_checkbox_("ui.A_Checkbox", false, true),
      an_int_no_input_("ui.An_Int_No_Input", 2),
      save_window_("ui.Save_Window", false, false),
      save_cube_("ui.Save_Cube", false, false),
      record_cube_("ui.Record_Cube", false, false) {}

spPangolinScenegraphGui::~spPangolinScenegraphGui() {}

void spPangolinScenegraphGui::InitGui() {
  if (!GeneralTools::CheckFileExists(SPIRITGUI_PARAM_FILE)) {
    std::cerr << "Error: Missing Pangolin config file." << std::endl;
  }

  // Load configuration data
  pangolin::ParseVarsFile(SPIRITGUI_PARAM_FILE);

  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind(SPIRITGUI_WINDOW_NAME, 640, 480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  s_cam_ = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
      pangolin::ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pangolin::AxisY));

  // Add named OpenGL viewport to window and provide 3D Handler
  d_cam_ = pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH_), 1.0,
                         -640.0f / 480.0f)
              .SetHandler(new pangolin::Handler3D(s_cam_));

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH_));

  // Demonstration of how we can register a keyboard hook to alter a Var
  pangolin::RegisterKeyPressCallback(
      pangolin::PANGO_CTRL + 'b',
      pangolin::SetVarFunctor<double>("ui.A Double", 3.5));

  // Demonstration of how we can register a keyboard hook to trigger a method
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r',
                                     this->SampleMethod);
}

bool spPangolinScenegraphGui::ShouldQuit() { return (pangolin::ShouldQuit()); }

void spPangolinScenegraphGui::Refresh() {
  // Clear entire screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Activate efficiently by object
  d_cam_.Activate(s_cam_);

  // Render some stuff
  glColor3f(1.0, 1.0, 1.0);
  pangolin::glDrawColouredCube();

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

  if (pangolin::Pushed(save_cube_)) {
    d_cam_.SaveOnRender("cube");
  }

  if (pangolin::Pushed(record_cube_)) {
    pangolin::DisplayBase().RecordOnRender(
        "ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");
  }
}

void spPangolinScenegraphGui::SampleMethod() {
  std::cout << "sample Method of spPangolinScenegraph is called ..."
            << std::endl;
}
