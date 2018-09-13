#ifndef SP_MESH_H__
#define SP_MESH_H__

#include <spirit/Objects/spCommonObject.h>
#include <spirit/spMeshVisitor.h>

class spMesh : public spCommonObject {
 public:
  spMesh(const osg::ref_ptr<osg::Node> &meshnode);
  ~spMesh();
  void SetPose(const spPose& pose);
  const spPose& GetPose();
  void SetColor(const spColor& color);
  const spColor& GetColor();

  bool IsDynamic();

  void SetDimensions(const spMeshSize& dims);
  spMeshSize GetDimensions();

  osg::ref_ptr<osg::Node> GetMesh();


  Eigen::VectorXd localSearch(Eigen::MatrixXd vertex, double x, double y, double z, double r);
  Eigen::VectorXd medianAngles(Eigen::MatrixXd normal, Eigen::VectorXd index);
  double checkPitch(Eigen::VectorXd angles);
  Eigen::VectorXd nearestVertex(Eigen::MatrixXd vertex, Eigen::VectorXd carPose);
  Eigen::VectorXd checkPoint(Eigen::VectorXd v, Eigen::VectorXd carPose, double rw);

 private:
  spMeshSize dims_;
  spPose pose_;
  spColor color_;
  double mass_;
  osg::ref_ptr<osg::Node> mesh_;
  //spMeshVisitor nodeinfo_;
  //Eigen::MatrixXd vertexdata_;
  //Eigen::MatrixXd normaldata_;
};

#endif  //  SP_MESH_H__
