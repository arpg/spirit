#include "spirit/Gui.h"

///////////////////////////////////////////////////////////////////////////////
SpiritGui::SpiritGui()
    : render_state_(
          pangolin::ProjectionMatrix(WINDOW_WIDTH, WINDOW_HEIGHT, 420, 420,
                                     WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0.1,
                                     1000),
          pangolin::ModelViewLookAt(5, 5, 5, 0, 0, 0, pangolin::AxisNegZ)),
      viewType_(eNeutral),
      selectedWaypoint_(-1) {
  pangolin::CreateGlutWindowAndBind("Main", WINDOW_WIDTH, WINDOW_HEIGHT);
  glewInit();
}

///////////////////////////////////////////////////////////////////////////////
SpiritGui::~SpiritGui() {
  // delete all status texts
  for (size_t ii = 0; ii < statusLines_.size(); ii++) {
    delete statusLines_[ii];
  }

//  for (size_t ii = 0; ii < cars_.size(); ii++) {
//    delete cars_[ii];
//  }

  for (size_t ii = 0; ii < waypoints_.size(); ii++) {
    delete waypoints_[ii];
  }
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::Render() {
  // boost::mutex::scoped_lock lock(m_DrawMutex);
  view_->ActivateScissorAndClear();

//  if (followCar_ != NULL) {
//    render_state_.Follow(OpenGlMatrix(followCar_->m_GLCar.GetPose4x4_po()));
//  }

  // TODO: remove this and debug the GL problem that's occuring
  glGetError();

  // Swap frames and Process Events
  pangolin::FinishGlutFrame();

  // handle waypoints
  for (size_t ii = 0; ii < waypoints_.size(); ii++) {
    if (waypoints_[ii]->m_Waypoint.m_bPendingActive == true) {
      selectedWaypoint_ = ii;
      waypoints_[ii]->m_Waypoint.m_bPendingActive = false;
      waypoints_[ii]->m_Waypoint.m_bActive = true;
      break;
    }
  }

  if (selectedWaypoint_ != -1) {
    for (size_t ii = 0; ii < waypoints_.size(); ii++) {
      if ((int)ii != selectedWaypoint_) {
        waypoints_[ii]->m_Waypoint.m_bActive = false;
        waypoints_[ii]->m_Waypoint.m_bPendingActive = false;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::Init(const std::string sTerrainMeshFileName, SceneGraph::GLMesh* pMesh,
                         const bool bViconCoords /* = false */) {
  const aiScene* pScene = aiImportFile(
      sTerrainMeshFileName.c_str(),
      aiProcess_Triangulate | aiProcess_GenSmoothNormals |
          aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes |
          aiProcess_FindInvalidData | aiProcess_FixInfacingNormals);
  std::cout << aiGetErrorString() << std::endl;

  if (bViconCoords) {
    pScene->mRootNode->mTransformation =
        aiMatrix4x4(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);
  }
  pMesh->Init(pScene);
  Init(pMesh);
}

void SpiritGui::AddGLObject(SceneGraph::GLObject* object, bool cast_shadows) {
  scene_graph_.AddChild(object);
  light_->AddShadowCaster(object);
}

void SpiritGui::Add2DGLObject(SceneGraph::GLObject* object) {
  scene_graph2d_.AddChild(object);
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::Init(SceneGraph::GLObject* terrain) {
  terrain_ = terrain;

  scene_graph_.ApplyPreferredGlSettings();
  scene_graph2d_.ApplyPreferredGlSettings();
  scene_graph_widgets_.ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);

  // add the lights
  light_ = new SceneGraph::GLShadowLight(100, 100, -100, 1024, 1024);
  static_light_ = new SceneGraph::GLShadowLight(100, 100, -100, 4096, 4096);
  light_->SetShadowsEnabled(false);
  static_light_->SetShadowsEnabled(false);
  light_->AddShadowReceiver(terrain_);
  static_light_->AddShadowCasterAndReceiver(terrain_);

  CheckForGLErrors();

  static_light_->SetAmbient(Eigen::Vector4f(0.1, 0.1, 0.1, 1.0));
  static_light_->SetDiffuse(Eigen::Vector4f(0.4, 0.4, 0.4, 1.0));
  light_->SetAmbient(Eigen::Vector4f(0.1, 0.1, 0.1, 1.0));
  light_->SetDiffuse(Eigen::Vector4f(0.4, 0.4, 0.4, 1.0));
  scene_graph_.AddChild(static_light_);
  scene_graph_.AddChild(light_);

  view_ =
      &pangolin::CreateDisplay()
           .SetBounds(0.0, 1.0, 0, 1.0)
           .SetHandler(new SceneGraph::HandlerSceneGraph(
               scene_graph_, render_state_, pangolin::AxisNegZ, 0.01f))
           .SetDrawFunction(SceneGraph::ActivateScissorClearDrawFunctor3d2d(
               scene_graph_, render_state_, scene_graph2d_, render_state2d_));

//  panelview_ =
//      &pangolin::CreateDisplay()
//           .SetBounds(1.0 - (double)UI_PANEL_HEIGHT / (double)WINDOW_HEIGHT,
//                      1.0, 0, (double)UI_PANEL_WIDTH / (double)WINDOW_WIDTH)
//           .SetHandler(new PlannerHandler(scene_graph_widgets_, render_state2d_,
//                                          pangolin::AxisNegZ, 0.01f,
//                                          &widgetPanels_))
//           .SetDrawFunction(ActivateScissorBlendedDrawFunctor(
//               scene_graph_widgets_, render_state2d_));

  pangolin::RegisterKeyPressCallback(
      'v', std::bind(&SpiritGui::_CommandHandler, this, eChangeView));
  pangolin::RegisterKeyPressCallback(
      '+', std::bind(&SpiritGui::_CommandHandler, this, eIncreaseWpVel));
  pangolin::RegisterKeyPressCallback(
      '-', std::bind(&SpiritGui::_CommandHandler, this, eDecreaseWpVel));
  pangolin::RegisterKeyPressCallback(
      'S', [this] { this->panelview_->Show(!this->panelview_->IsShown()); });

  _PopulateSceneGraph();

  CheckForGLErrors();
}

///////////////////////////////////////////////////////////////////////////////
//void WaypointerGui::AddPanel(GLWidgetPanel* panel) {
//  widgetPanels_.push_back(panel);
//  panel->Init(UI_PANEL_WIDTH, UI_PANEL_HEIGHT);
//  scene_graph_widgets_.AddChild(panel);
//}

///////////////////////////////////////////////////////////////////////////////
int SpiritGui::AddStatusLine(StatusLineLocation location) {
  // first find the last status line in this location
  StatusLine* pLastStatus = 0;
//  for (size_t ii = 0; ii < statusLines_.size(); ii++) {
//    if (statusLines_[ii]->m_Location == location) {
//      pLastStatus = statusLines_[ii];
//    }
//  }

  Eigen::Vector2d pos(0, 0);
  // if the last status is null, initialize its position
  if (pLastStatus == NULL) {
    switch (location) {
      case eTopLeft:
        pos << 0.01, 0.975;
        break;

      case eBottomLeft:
        pos << 0.01, 0.025;
        break;
    }
  } else {
//    pos = pLastStatus->m_dPos2D;
    switch (location) {
      case eTopLeft:
        pos[1] -= 0.025;
        break;

      case eBottomLeft:
        pos[1] -= 0.025;
        break;
    }
  }

  // and now create a new status line
  statusLines_.push_back(new StatusLine());

  StatusLine* statusLine = statusLines_.back();
//  statusLine->m_dPos2D = pos;
//  statusLine->m_Location = location;
//  statusLine->m_GLText.SetPosition(pos[0], pos[1]);

  // boost::mutex::scoped_lock lock(m_DrawMutex);
//  scene_graph2d_.AddChild(&statusLine->m_GLText);

  return statusLines_.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::SetStatusLineText(int id, std::string text) {
  // boost::mutex::scoped_lock lock(m_DrawMutex);
//  statusLines_[id]->m_GLText.SetText(text);
}

///////////////////////////////////////////////////////////////////////////////
int SpiritGui::AddCar(const double& wheel_base, const double& width) {
//  cars_.push_back(new Car());
//  Car* pCar = cars_.back();

//  pCar->m_GLCar.Init(eMesh);
//  pCar->m_GLCar.SetCarScale(Eigen::Vector3d(wheel_base, width, wheel_base));

//  pCar->m_CarLineSegments.SetColor(GLColor(0.0f, 0.0f, 1.0f));

//  // add this to the scenegraph
//  // boost::mutex::scoped_lock lock(m_DrawMutex);

//  scene_graph_.AddChild(&pCar->m_GLCar);
//  light_->AddShadowCasterAndReceiver(&pCar->m_GLCar);
//  static_light_->AddShadowReceiver(&pCar->m_GLCar);
//  for (size_t ii = 0; ii < pCar->m_GLCar.GetWheels().size(); ii++) {
//    scene_graph_.AddChild(pCar->m_GLCar.GetWheels()[ii]);
//  }

//  // add the trajectory to the scenegraph
//  scene_graph_.AddChild(&pCar->m_CarLineSegments);

//  scene_graph_.AddChild(&pCar->m_Axis);

  //  return cars_.size() - 1;
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::SetCarState(const int& id, const VehicleState& state,
                                bool add2trajectory /* = false */) {
//  Car* pCar = cars_[id];
//  std::unique_lock<std::mutex> lock(*pCar, std::try_to_lock);

//  Sophus::SE3d state_aug = state.m_dTwv;
//  state_aug.translation() -= GetBasisVector(state_aug, 2) * 0.05;
//  pCar->m_GLCar.SetPose(state_aug.matrix());

//  Sophus::SE3d axisPose = state.m_dTwv;
//  VehicleState::AlignWithVelocityVector(axisPose, state.m_dV);
//  pCar->m_Axis.SetPose(axisPose.matrix());
//  pCar->m_Axis.SetAxisSize(state.m_dV.norm());

//  for (size_t ii = 0; ii < state.m_vWheelStates.size(); ii++) {
//    Sophus::SE3d T = state.m_vWheelStates[ii];
//    pCar->m_GLCar.SetRelativeWheelPose(ii, T);
//  }

//  // add this pose to the trajectory of the car if required
//  if (add2trajectory) {
//    pCar->m_CarLineSegments.AddVertex(state.m_dTwv.translation());
//  }
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::SetCarVisibility(const int& id, const bool& visible) {
//  Car* pCar = cars_[id];
//  pCar->m_GLCar.SetVisible(visible);
//  for (size_t ii = 0; ii < pCar->m_GLCar.GetWheels().size(); ii++) {
//    pCar->m_GLCar.GetWheels()[ii]->SetVisible(visible);
//  }
}

///////////////////////////////////////////////////////////////////////////////
int SpiritGui::AddWaypoint(const Eigen::Vector6d& pose,
                               const double& velocity) {
  waypoints_.push_back(new Waypoint());
  Waypoint* pWaypoint = waypoints_.back();

  pWaypoint->m_Waypoint.SetPose(pose);
  pWaypoint->m_Waypoint.SetVelocity(velocity);
  pWaypoint->m_Waypoint.SetDirty(true);

  // boost::mutex::scoped_lock lock(m_DrawMutex);
  scene_graph_.AddChild(&pWaypoint->m_Waypoint);

  return waypoints_.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::ClearWaypoints() {
  // remove all waypoints from the vector and from the scenegraph
  for (size_t ii = 0; ii < waypoints_.size(); ii++) {
    scene_graph_.RemoveChild(&waypoints_[ii]->m_Waypoint);
    delete waypoints_[ii];
  }
  waypoints_.clear();
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::SetWaypointDirtyFlag(bool flag) {
  for (size_t ii = 0; ii < waypoints_.size(); ii++) {
    waypoints_[ii]->m_Waypoint.SetDirty(flag);
  }
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::_PopulateSceneGraph() {
  // add the terrain
  scene_graph_.AddChild(terrain_);

  // add a grid
  scene_graph_.AddChild(new SceneGraph::GLGrid());
}

///////////////////////////////////////////////////////////////////////////////
void SpiritGui::_SetViewType(const PlannerViewType& view_type) {
  // get the car position
  Eigen::Vector4d target(0, 0, 0, 1);
  Eigen::Vector4d nearSource(-0.5, 0, -0.15, 1);
  Eigen::Vector4d farSource(-1, 0, -0.15, 1);
  Eigen::Vector4d wheelSource(-0.5, -0.25, -0.15, 1);

  // transform the sources based on where the car is
//  Car* pCar = cars_[0];
//  target = pCar->m_GLCar.GetPose4x4_po() * target;
//  nearSource = pCar->m_GLCar.GetPose4x4_po() * nearSource;
//  farSource = pCar->m_GLCar.GetPose4x4_po() * farSource;
//  wheelSource = pCar->m_GLCar.GetPose4x4_po() * wheelSource;

  switch (view_type) {
    case eNeutral:
//      SetFollowCar(-1);
      render_state_.Unfollow();
      break;

    case eFollowNear:
      // set the model view matrix
      render_state_.SetModelViewMatrix(pangolin::ModelViewLookAt(
          nearSource[0], nearSource[1], nearSource[2], target[0], target[1],
          target[2], pangolin::AxisNegZ));
//      SetFollowCar(0);
      break;

    case eFollowFar:
      // set the model view matrix
      render_state_.SetModelViewMatrix(pangolin::ModelViewLookAt(
          farSource[0], farSource[1], farSource[2], target[0], target[1],
          target[2], pangolin::AxisNegZ));
//      SetFollowCar(0);
      break;

    case eFollowWheel:
      // set the model view matrix
      render_state_.SetModelViewMatrix(pangolin::ModelViewLookAt(
          wheelSource[0], wheelSource[1], wheelSource[2], target[0], target[1],
          target[2], pangolin::AxisNegZ));
//      SetFollowCar(0);
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
//void WaypointerGui::_CommandHandler(const WaypointerGuiCommands& command) {
//  switch (command) {
//    case eChangeView:
//      // advance the view type
//      viewType_ = viewType_ == eFollowWheel ? eNeutral
//                                            : (PlannerViewType)(viewType_ + 1);
//      // and now set this view
//      _SetViewType(viewType_);
//      break;

//    case eVideoToggle:
//      //"mencoder &fifo_file -demuxer rawvideo  -rawvideo
//      //fpgs=20:w=$w:h=$h:format=rgb24 -o &output_file -ovc lavc -lavcopts
//      //vcodec=msmpeg4v2:vbitrate=12000"
//      // create file with this script
//      // chmod the properties to executable 77
//      // create command string with script name + variables
//      // pipe output to dev null (in the string)
//      // use system command to call mencoder

//      // hold space for openGL buffer
//      // glPixelStorei(GL_U
//      // glReadBuffer(GL_BACK)
//      // glReadPixels
//      break;

//    case eIncreaseWpVel:
//      if (selectedWaypoint_ != -1) {
//        waypoints_[selectedWaypoint_]->m_Waypoint.SetVelocity(
//            waypoints_[selectedWaypoint_]->m_Waypoint.GetVelocity() * 1.05);
//      }
//      break;

//    case eDecreaseWpVel:
//      if (selectedWaypoint_ != -1) {
//        waypoints_[selectedWaypoint_]->m_Waypoint.SetVelocity(
//            waypoints_[selectedWaypoint_]->m_Waypoint.GetVelocity() * 0.95);
//      }
//      break;
//  }
//}
