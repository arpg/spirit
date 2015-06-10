#ifndef GUI_H
#define GUI_H

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT

#include <stdio.h>
#include <vector>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <sophus/sophus.hpp>

class SpiritGui {
 public:

  SpiritGui();
  ~SpiritGui();

  // Initialize the GUI variables
  void Init(void);

  // Render a new  frame
  bool Render(void);

  void AddWaypoint(void);
  void AddGroundMesh(std::string file_name);
  int AddAxis(Eigen::Vector6d T_w_a);
  int NumOfAxes(void);
  void DelAxis(int axis_num);

 private:
  // GUI variables
  SceneGraph::GLSceneGraph glgraph_;
  SceneGraph::GLLight* gllight_;
  SceneGraph::GLGrid* glgrid_;
  SceneGraph::GLMesh glgroundmesh_;
  pangolin::View view3d_;
  SceneGraph::HandlerSceneGraph* handler_sg_;
  // Object Variables
  struct Waypoint {
    Waypoint() : type(eWaypoint_Normal) {}
    WaypointType type;
    SceneGraph::GLWayPoint glwaypoint;
  };
  std::vector<Waypoint>   waypoints_vec_;
  std::vector<SceneGraph::GLAxis> axes_vec_;


};
#endif
