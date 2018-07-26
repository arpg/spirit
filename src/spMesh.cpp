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
  meshnode->accept(nodeinfo_);
  Eigen::MatrixXd vertexdata_(nodeinfo_.vertices->size(), 3);
  Eigen::MatrixXd normaldata_(nodeinfo_.normals->size(), 3);
}

spMesh::~spMesh() {}

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

osg::ref_ptr<osg::Node> spMesh::GetMesh(){
  return mesh_;
}

Eigen::MatrixXd spMesh::GetVertices(){
  for(unsigned int ii = 0; ii < nodeinfo_.vertices->size(); ii++){
     vertexdata_(ii,0) = (*nodeinfo_.vertices)[ii][0];
     vertexdata_(ii,1) = (*nodeinfo_.vertices)[ii][1];
     vertexdata_(ii,2) = (*nodeinfo_.vertices)[ii][2];
    }
   return vertexdata_;
}

Eigen::MatrixXd spMesh::GetNormals(){
  for(unsigned int ii = 0; ii < nodeinfo_.normals->size(); ii++){
     normaldata_(ii,0) = (*nodeinfo_.normals)[ii][0];
     normaldata_(ii,1) = (*nodeinfo_.normals)[ii][1];
     normaldata_(ii,2) = (*nodeinfo_.normals)[ii][2];
    }
  return normaldata_;
}




















