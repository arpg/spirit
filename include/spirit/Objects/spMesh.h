#ifndef SP_MESH_H__
#define SP_MESH_H__

#include <spirit/Objects/spCommonObject.h>

/// spMesh to be implemented later, for now we are gonna make a flat surface as
/// a ground for spirit
class spMesh : public spCommonObject {
 public:
  spMesh();
  ~spMesh();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();

  bool IsDynamic();

  void SetDimensions(const spMeshSize& dims);
  spMeshSize GetDimensions();

 private:
  spMeshSize dims_;
  spPose pose_;
  spColor color_;
  double mass_;
};

#endif  //  SP_MESH_H__
