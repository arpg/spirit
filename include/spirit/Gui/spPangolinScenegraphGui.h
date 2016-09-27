#ifndef SP_PANGOLINSCENEGRAPHGUI_H__
#define SP_PANGOLINSCENEGRAPHGUI_H__

#include <spirit/spSettings.h>
#include <spirit/Gui/spCommonGui.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <spirit/Types/spTypes.h>
#include <spirit/spGeneralTools.h>

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
  void AddWaypoint(spWaypoint& waypoint);
  void AddVehicle(spVehicle& vehicle);
  void UpdateGuiObjectsFromSpirit(Objects &spobj);
  void UpdateSpiritObjectsFromGui(Objects& spobjects);

 private:
  static void KeyActionMethodSample();
  void UpdateBoxGuiObject(spBox& spobj);
  void UpdateWaypointGuiObject(spWaypoint& spobj);
  void UpdateVehicleGuiObject(spVehicle& spobj);

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
