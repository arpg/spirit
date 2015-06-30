#ifndef GUI_H_
#define GUI_H_

#include <spirit/objects/SpiritAxes.h>
#include <spirit/objects/SpiritWaypoints.h>
#include <spirit/objects/CommonObj.h>
#include <spirit/objects/SpiritStaticTerrain.h>
#include <spirit/objects/SpiritCars.h>

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define UI_PANEL_WIDTH 200
#define UI_PANEL_HEIGHT WINDOW_HEIGHT


class SpiritGui {
 public:

  SpiritGui();
  ~SpiritGui();

  // Render a new  frame
  bool Render(void);

  SpiritAxes axes_;
  SpiritWaypoints waypoints_;
  // create the ground mesh
  SpiritStaticTerrain groundmesh_;
  // Initialize the collision shape for the car
  SpiritCars cars_;

 private:
  // GUI variables
  SceneGraph::GLSceneGraph glgraph_;
  SceneGraph::GLLight* gllight_;
  SceneGraph::GLGrid* glgrid_;
  pangolin::OpenGlRenderState glrenderstate_;
  pangolin::View view3d_;
  SceneGraph::HandlerSceneGraph* handler_sg_;
};
#endif  //GUI_H_
