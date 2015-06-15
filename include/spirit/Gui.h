#ifndef GUI_H_
#define GUI_H_

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT

#include <stdio.h>
#include <list>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <sophus/sophus.hpp>
#include <Eigen/Eigen>
#include <spirit/objects/SpiritAxes.h>
#include <spirit/objects/SpiritWaypoints.h>
#include <spirit/objects/CommonObj.h>

class SpiritGui {
 public:

  SpiritGui();
  ~SpiritGui();

  // Render a new  frame
  bool Render(void);

  void AddWaypoint(void);
  void AddGroundMesh(std::string file_name);

  SpiritAxes axes_;
  SpiritWaypoints waypoints_;


 private:
  // GUI variables
  SceneGraph::GLSceneGraph glgraph_;
  SceneGraph::GLLight* gllight_;
  SceneGraph::GLGrid* glgrid_;
  SceneGraph::GLMesh glgroundmesh_;
  pangolin::OpenGlRenderState glrenderstate_;
  pangolin::View view3d_;
  SceneGraph::HandlerSceneGraph* handler_sg_;
};
#endif  //GUI_H_
