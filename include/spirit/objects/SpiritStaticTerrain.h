#ifndef SPIRIT_STATICTERRAIN_H_
#define SPIRIT_STATICTERRAIN_H_

#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <spirit/objects/CommonObj.h>

class SpiritStaticTerrain : public SpiritCommonObj {
 public:
  SpiritStaticTerrain(SceneGraph::GLSceneGraph& graph);
  ~SpiritStaticTerrain();

  int AddObj(Eigen::Vector6d T_w_a);
  int NumOfObjs();
  int DelObj(int objnum);
  void SetMeshFilePath(std::string file_name);
  btCollisionShape* GetCollisionShape();

 private:
  // glgraph to be updated
  SceneGraph::GLSceneGraph* glgraph_;

  btVector3 dMin_;
  btVector3 dMax_;
  std::string mesh_file_path;
  const aiScene* pScene;
  SceneGraph::GLMesh glmesh_;
  btTriangleMesh triangle_mesh_;
  btCollisionShape* collision_shape_;
  SceneGraph::GLShadowLight *glshadowlight_;
  SceneGraph::GLShadowLight *glstaticlight_;

};

#endif  // SPIRIT_STATICTERRAIN_H_
