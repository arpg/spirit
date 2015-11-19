#ifndef SPIRIT_STATICTERRAIN_H_
#define SPIRIT_STATICTERRAIN_H_

#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <spirit/objects/CommonObj.h>

DECLARE_string(mesh);

class SpiritStaticTerrain : public SpiritCommonObj {
 public:
  SpiritStaticTerrain(SceneGraph::GLSceneGraph& graph);
  ~SpiritStaticTerrain();

  int AddObj(Eigen::Vector6d T_w_a);
  int NumOfObjs();
  int DelObj(int objnum);
  void SetMeshFilePath();
  btCollisionShape* GetCollisionShape();

  void Clear() { for(size_t ii = 0; ii < NumOfObjs() ; ii++ ) { DelObj(ii); } }


private:
  // glgraph to be updated
  SceneGraph::GLSceneGraph* glgraph_;

  btVector3 dMin_;
  btVector3 dMax_;
  std::string mesh_file_path;
  const aiScene* pScene;
  SceneGraph::GLMesh glmesh_;
  SceneGraph::GLColor mesh_color_;
  btTriangleMesh triangle_mesh_;
  btCollisionShape* collision_shape_;
  SceneGraph::GLShadowLight *glshadowlight_;
  SceneGraph::GLShadowLight *glstaticlight_;

};

#endif  // SPIRIT_STATICTERRAIN_H_
