#include <spirit/Objects/spMesh.h>

spMesh::spMesh() {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  index_phy_ = -1;
  index_gui_ = -1;
  obj_phychanged_ = false;
  obj_guichanged_ = false;
  is_gui_modifiable_ = false;
  object_type_ = spObjectType::BOX;
}

spMesh::~spMesh() {}

void spMesh::SetDimensions(const spMeshSize& dims) {
  dims_ = dims;
  obj_guichanged_ = true;
  obj_phychanged_ = true;
}

spMeshSize spMesh::GetDimensions() {
  return dims_;
}


void spMesh::SetPose(const spPose& pose) {
  pose_ = pose;
  obj_phychanged_ = true;
  obj_guichanged_ = true;
}

const spPose& spMesh::GetPose(){
  return pose_;
}

void spMesh::SetColor(const spColor& color) {
  color_ = color;
  obj_guichanged_ = true;
}

const spColor& spMesh::GetColor() {
  return color_;
}

bool spMesh::IsDynamic() {
  return false;
}
