#include <spirit/Objects/spMesh.h>

spMesh::spMesh(const osg::ref_ptr<osg::Node>& meshnode) {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::MESH;

  // for OSG
  mesh_ = meshnode;
  //nodeinfo_ = nodeinfo;
  //vertexdata_(nodeinfo.vertices->size(), 3);
  //normaldata_(nodeinfo.normals->size(), 3);
}

spMesh::~spMesh() {}

osg::ref_ptr<osg::Node> spMesh::GetMesh(){
    return mesh_;
}

void spMesh::SetDimensions(const spMeshSize& dims) {
  dims_ = dims;
  obj_guichanged_ = true;
}

spMeshSize spMesh::GetDimensions() {
  return dims_;
}


void spMesh::SetPose(const spPose& pose) {
  pose_ = pose;
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





















