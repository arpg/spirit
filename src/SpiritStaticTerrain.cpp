#include <spirit/objects/SpiritStaticTerrain.h>

DEFINE_string(mesh, "", "Defines the mesh of the environment.");

SpiritStaticTerrain::SpiritStaticTerrain(SceneGraph::GLSceneGraph& graph)
    : glgraph_(&graph),
      // green color is (0.4f,0.5f,0.4f)
      mesh_color_(SceneGraph::GLColor(1.0f, 1.0f, 1.0f)) {
  collision_shape_ = nullptr;
}

SpiritStaticTerrain::~SpiritStaticTerrain() {
  delete pScene;
  delete glgraph_;
  delete collision_shape_;
  delete glshadowlight_;
  delete glstaticlight_;
}

void SpiritStaticTerrain::SetMeshFilePath() {
  mesh_file_path = FLAGS_mesh;
}

int SpiritStaticTerrain::AddObj(Eigen::Vector6d T_w_a) {
  // TODO(sina) : addobj sometime crashes, there is a problem with pointers.
  //              when disabling globj part the other part works and vs.

  if (FLAGS_mesh.empty()) {
    std::cerr << "Meshfile path has not been specified."
                 " specify it by calling SetMeshFilePath()" << std::endl;
    return -1;
  }
  // create and add scenegraph object
  pScene = aiImportFile(
      FLAGS_mesh.c_str(),
      aiProcess_Triangulate | aiProcess_GenSmoothNormals |
          aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes |
          aiProcess_FindInvalidData | aiProcess_FixInfacingNormals);

  if (pScene == NULL) {
    throw SceneGraph::GLMeshException("unable to load mesh");
    return -1;
  }

  // Create collision shape
  pScene->mRootNode->mTransformation =
//      aiMatrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
      aiMatrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);
  glmesh_.Init(pScene);
  glmesh_.SetMeshColor(mesh_color_);
  glmesh_.SetPose(T_w_a);
  glgraph_->AddChild(&glmesh_);
  // add the lights
  glgraph_->ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 0);
  glshadowlight_ = new SceneGraph::GLShadowLight(100, 100, -100, 1024, 1024);
  glstaticlight_ = new SceneGraph::GLShadowLight(100, 100, -100, 4096, 4096);
  glshadowlight_->SetShadowsEnabled(false);
  glstaticlight_->SetShadowsEnabled(false);
  glshadowlight_->AddShadowReceiver(&glmesh_);
  glstaticlight_->AddShadowCasterAndReceiver(&glmesh_);
  glstaticlight_->SetAmbient(Eigen::Vector4f(0.1, 0.1, 0.1, 1.0));
  glstaticlight_->SetDiffuse(Eigen::Vector4f(0.4, 0.4, 0.4, 1.0));
  glshadowlight_->SetAmbient(Eigen::Vector4f(0.1, 0.1, 0.1, 1.0));
  glshadowlight_->SetDiffuse(Eigen::Vector4f(0.4, 0.4, 0.4, 1.0));
  glgraph_->AddChild(glstaticlight_);
  glgraph_->AddChild(glshadowlight_);

  // Using pTriangleMesh and the terrain mesh, fill in the gaps to create a
  // static hull.
  BulletCarModel::GenerateStaticHull(pScene, pScene->mRootNode,
                                     pScene->mRootNode->mTransformation, 1.0,
                                     triangle_mesh_, dMin_, dMax_);
  // Generate the collision shape from the triangle mesh --- to know where the
  // ground is.
  collision_shape_ = new btBvhTriangleMeshShape(&triangle_mesh_, true, true);

  return 1;
}

int SpiritStaticTerrain::NumOfObjs() { return 1; }

int SpiritStaticTerrain::DelObj(int objnum) {
  glgraph_->RemoveChild(&glmesh_);
  return 0;
}

btCollisionShape* SpiritStaticTerrain::GetCollisionShape() {
  if (collision_shape_ != nullptr) {
    return collision_shape_;
  } else {
    std::cerr << "Error : collision shape has not been set" << std::endl;
  }
}
