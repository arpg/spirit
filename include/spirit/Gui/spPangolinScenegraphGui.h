#ifndef SP_PANGOLINSCENEGRAPHGUI_H__
#define SP_PANGOLINSCENEGRAPHGUI_H__

#include <spirit/Settings.h>
#include <spirit/Gui/spCommonGui.h>
#include <pangolin/pangolin.h>
#include <spirit/GeneralTools.h>

/// this class is the interface between Pangolin/Scenegraph and spGui
class spPangolinScenegraphGui : public spCommonGUI {
 public:
  spPangolinScenegraphGui();
  ~spPangolinScenegraphGui();

  void InitGui();
  bool ShouldQuit();
  void Refresh();
  void CheckKeyboardAction();

 private:
  static void SampleMethod();

  pangolin::OpenGlRenderState s_cam_;
  pangolin::View d_cam_;
  const int UI_WIDTH_ = 180;

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
  pangolin::Var<bool> save_cube_;
  pangolin::Var<bool> record_cube_;

};

#endif  // SP_PANGOLINSCENEGRAPHGUI_H__
