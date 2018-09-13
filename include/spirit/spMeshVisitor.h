#ifndef SP_MESHVISITOR_H
#define SP_MESHVISITOR_H
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>
#include <iostream>
#include <eigen3/Eigen/Eigen>

class spMeshVisitor : public osg::NodeVisitor
{
public:
    osg::Vec3Array* vertices; // pointer to vetices in .ply file
    osg::Vec3Array* normals; // pointer to normal vectors in .ply file

    spMeshVisitor(): _level(0)
    { setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN); }
    std::string spaces()
    { return std::string(_level*2, ' '); }

    virtual void apply( osg::Node& node );
    virtual void apply( osg::Geode& geode );

    void GetMeshData();
    Eigen::MatrixXd GetVertices();
    Eigen::MatrixXd GetNormals();
    Eigen::MatrixXd BoundingBoxVertex(Eigen::MatrixXd vertex, Eigen::VectorXd bounds);

    struct meshstruct{
        Eigen::MatrixXd *vtx_ptr;
        Eigen::MatrixXd *nrml_ptr;

        // bounded vertex and normal data
        Eigen::MatrixXd *bvtx_ptr;
        Eigen::MatrixXd *bnrml_ptr;

    };
    meshstruct mstruct;

protected:
    unsigned int _level;


};

#endif // SP_MESHVISITOR_H
