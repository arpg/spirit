#ifndef GUI_H
#define GUI_H

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT

#include <stdio.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
//#include <MochaGui/GLCar.h>
//#include <MochaGui/GLWidgetPanel.h
//#include <MochaGui/UiCommon.h>

class SpiritGui {
 public:
//  enum PlannerViewType { eNeutral, eFollowNear, eFollowFar, eFollowWheel };

//  enum PlannerGuiCommands {
//    eChangeView,
//    eVideoToggle,
//    eIncreaseWpVel,
//    eDecreaseWpVel,
//  };

//  enum StatusLineLocation { eTopLeft = 0, eBottomLeft = 1 };

//  struct StatusLine {
//    StatusLineLocation location;
//    SceneGraph::GLText gltext;
//    Eigen::Vector2d pos2d;
//  };

  struct Waypoint {
    Waypoint() : m_eType(eWaypoint_Normal) {}
    SceneGraph::GLWayPoint waypoint;
    WaypointType m_eType;
  };

  SpiritGui();
  ~SpiritGui();

  /// Render a frame
  void Render();

  /// Initializes the GUI.
  void Init(SceneGraph::GLObject* terrain  //< The GLObject for the terrain.
            // This will get added to the
            // scenegraph and rendered
            );
  void Init(const std::string sTerrainMeshFileName, SceneGraph::GLMesh* pMesh,
            const bool bViconCoords = false);

  /// Adds a new GLObject to the scenegraph
  void AddGLObject(SceneGraph::GLObject* object, bool cast_shadows = false);
  /// Adds a new 2D object to the scenegraph
  void Add2DGLObject(SceneGraph::GLObject* object);
  /// Adds a GLPanel object to the 2D scenegraph and hooks up its events
  //void AddPanel(GLWidgetPanel* panel);

  /// Adds a new status line to the gui, to allow the user to add text, Returns
  /// the ID of this status line
//  int AddStatusLine(StatusLineLocation location);
  /// Sets the text on a previously created status
  void SetStatusLineText(int id, std::string text);

  /// Add a new car to the scene. Returns the ID of the car
  int AddCar(const double& wheel_base, const double& width);
  /// Sets the position and rotation of the car and the wheels used the
  /// vehiclestate structure passed.
  void SetCarState(
      const int& id,
      const VehicleState&
          state,  //< The structure containing the state of the car and wheels
      bool add2trajectory = false  //< If true, a new point will be added to the
      // linestrip that tracks the car trajectory
      );
  /// Sets a car to follow, or if given an id of -1, will disable following
  //void SetFollowCar(const int& id) { followCar_ = id == -1 ? NULL : cars_[id]; }
  /// Sets the visibility flag of the chosen car
  void SetCarVisibility(const int& id, const bool& visible);

  /// Returns the current number of added waypoints
  int WaypointCount() { return waypoints_.size(); }
  /// Adds a new waypoint, returns the ID
  int AddWaypoint(const Eigen::Vector6d& pose, const double& velocity);
  /// Returns a pointer to the waypoint object
  Waypoint* GetWaypoint(const int& id) { return waypoints_[id]; }
  /// Deletes all waypoints
  void ClearWaypoints();
  /// Clears the car trajectory
  void ClearCarTrajectory(const int& id) {
//    cars_[id]->m_CarLineSegments.Clear();
  }
  /// Sets the dirty flag on all waypoints to true
  void SetWaypointDirtyFlag(bool flag);

 private:
  /// Populates the initial scenegraph items
  void _PopulateSceneGraph();

  /// Function which takes in keyboard commands and executes the corresponding
  /// action
//  void _CommandHandler(const PlannerGuiCommands& command);
  void _CommandHandler(const PlannerGuiCommands& command);

  /// Sets the camera as specified by the given view type
  void _SetViewType(const PlannerViewType& view_type);

  // objects
  SceneGraph::GLObject* terrain_;

  // view related variables
  SceneGraph::GLShadowLight* light_;
  SceneGraph::GLShadowLight* static_light_;
  SceneGraph::GLSceneGraph scene_graph_;
  SceneGraph::GLSceneGraph scene_graph_widgets_;
  SceneGraph::GLSceneGraph scene_graph2d_;
  pangolin::OpenGlRenderState render_state_;
  pangolin::OpenGlRenderState render_state2d_;
  pangolin::OpenGlRenderState render_state_widget_;
  pangolin::View* view_;
  pangolin::View* panelview_;

  // vector of text objects
  std::vector<Waypoint*> waypoints_;

  std::vector<StatusLine*> statusLines_;

  // vector of widgets panels
//  CPT-Application-Instructions-10302014.pdfstd::vector<GLWidgetPanel*> widgetPanels_;

  std::mutex drawMutex_;
  PlannerViewType viewType_;
  int selectedWaypoint_;
};

#endif  // GUI_H
