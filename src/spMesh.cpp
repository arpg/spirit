#include <spirit/Objects/spMesh.h>

spMesh::spMesh(const osg::ref_ptr<osg::Node> meshnode) {
  mass_ = 0;
  color_ = spColor(1,1,1);
  pose_ = spPose::Identity();
  index_gui_ = -1;
  obj_guichanged_ = false;
  modifiable_gui_ = false;
  object_type_ = spObjectType::BOX;

  // for OSG
  if(!meshnode){ SPERROREXIT("No mesh loaded");}
  mesh_ = meshnode;
  mesh_->accept(nodeinfo_);
  //std::cout<<nodeinfo_.vertices->size()<<std::endl;
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

const osg::ref_ptr<osg::Node> spMesh::GetMesh(){
  return mesh_;
}

Eigen::MatrixXd spMesh::GetVertices(){
  Eigen::MatrixXd vertexdata(nodeinfo_.vertices->size(), 3);
  for(unsigned int ii = 0; ii < nodeinfo_.vertices->size(); ii++){
     vertexdata(ii,0) = (*nodeinfo_.vertices)[ii][0];
     vertexdata(ii,1) = (*nodeinfo_.vertices)[ii][1];
     vertexdata(ii,2) = (*nodeinfo_.vertices)[ii][2];
    }
   return vertexdata;
}

Eigen::MatrixXd spMesh::GetNormals(){
  Eigen::MatrixXd normaldata(nodeinfo_.normals->size(), 3);
  for(unsigned int ii = 0; ii < nodeinfo_.normals->size(); ii++){
     normaldata(ii,0) = (*nodeinfo_.normals)[ii][0];
     normaldata(ii,1) = (*nodeinfo_.normals)[ii][1];
     normaldata(ii,2) = (*nodeinfo_.normals)[ii][2];
    }
  return normaldata;
}




















