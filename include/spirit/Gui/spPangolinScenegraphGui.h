#ifndef SP_PANGOLINSCENEGRAPHGUI_H__
#define SP_PANGOLINSCENEGRAPHGUI_H__

#include <spirit/spSettings.h>
#include <spirit/Gui/spCommonGui.h>
#include <pangolin/pangolin.h>
#include <spirit/spGeneralTools.h>
#include <SceneGraph/SceneGraph.h>

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT

/// this class is the interface between Pangolin/Scenegraph and spGui
class spPangolinScenegraphGui : public spCommonGUI {
 public:
  spPangolinScenegraphGui();
  ~spPangolinScenegraphGui();

  void InitGui();
  bool ShouldQuit();
  void Refresh();
  void CheckKeyboardAction();
  void AddBox(spBox& box);

 private:
  static void KeyActionMethodSample();

  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  pangolin::Var<bool> a_button_;
  pangolin::Var<double> a_double_;
  pangolin::Var<int> an_int_;
  pangolin::Var<double> a_double_log_;
  pangolin::Var<bool> a_checkbox_;
  pangolin::Var<int> an_int_no_input_;
  pangolin::Var<bool> save_window_;

  pangolin::OpenGlRenderState glrenderstate_;
  pangolin::View pangoview_;
  SceneGraph::GLSceneGraph glscenegraph_;
  SceneGraph::HandlerSceneGraph* handler_scenegraph_;
  std::vector<SceneGraph::GLObject*> globjects_;

};

#endif  // SP_PANGOLINSCENEGRAPHGUI_H__
