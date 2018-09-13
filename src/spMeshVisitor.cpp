#include "spirit/spMeshVisitor.h"

void spMeshVisitor::apply( osg::Node& node )
{
    //std::cout << spaces() << node.libraryName() << "::" << node.className() << std::endl;

    _level++;
    traverse( node );
    _level--;
}

void spMeshVisitor::apply( osg::Geode& geode )
{
    //std::cout << spaces() << geode.libraryName() << "::" << geode.className() << std::endl;
    _level++;

    osg::Geometry* geom = static_cast<osg::Geometry*>(geode.getDrawable(0));
    vertices = static_cast<osg::Vec3Array*>(geom->getVertexArray());
    normals = static_cast<osg::Vec3Array*>(geom->getNormalArray());

    for (unsigned int i=0; i<geode.getNumDrawables(); ++i)
    {
        osg::Drawable* drawable = geode.getDrawable(i);
        //std::cout << spaces() << drawable->libraryName() << "::" << drawable->className() << std::endl;
    }

    traverse( geode );
    _level--;
}

void spMeshVisitor::GetMeshData(){
   //Eigen::MatrixXd vertexdata(vertices->size(), 3);
   //Eigen::MatrixXd normaldata(normals->size(), 3);

   mstruct.vtx_ptr = new Eigen::MatrixXd(vertices->size(), 3);
   mstruct.nrml_ptr = new Eigen::MatrixXd(normals->size(), 3);

   for(unsigned int ii=0; ii<vertices->size(); ii++){
       /*
       vertexdata(ii,0) = (*vertices)[ii][0];
       vertexdata(ii,1) = (*vertices)[ii][1];
       vertexdata(ii,2) = (*vertices)[ii][2];

       normaldata(ii,0) = (*normals)[ii][0];
       normaldata(ii,1) = (*normals)[ii][1];
       normaldata(ii,2) = (*normals)[ii][2]; */

       (*mstruct.vtx_ptr)(ii,0) = (*vertices)[ii][0];
       (*mstruct.vtx_ptr)(ii,1) = (*vertices)[ii][1];
       (*mstruct.vtx_ptr)(ii,2) = (*vertices)[ii][2];

       (*mstruct.nrml_ptr)(ii,0) = (*normals)[ii][0];
       (*mstruct.nrml_ptr)(ii,1) = (*normals)[ii][1];
       (*mstruct.nrml_ptr)(ii,2) = (*normals)[ii][2];
   }
   //meshstruct_ptr.vtx_ptr = &vertexdata;
   //meshstruct_ptr.nrml_ptr = &normaldata;
   //std::cout<<meshstruct_ptr.vtx_ptr<<std::endl;

}

Eigen::MatrixXd spMeshVisitor::GetVertices(){
   Eigen::MatrixXd vertexdata(vertices->size(), 3);
   for(unsigned int ii=0; ii<vertices->size(); ii++){
       vertexdata(ii,0) = (*vertices)[ii][0];
       vertexdata(ii,1) = (*vertices)[ii][1];
       vertexdata(ii,2) = (*vertices)[ii][2];
   }
   return vertexdata;
}

Eigen::MatrixXd spMeshVisitor::GetNormals(){
   Eigen::MatrixXd normaldata(normals->size(), 3);
   for(unsigned int ii=0; ii<normals->size(); ii++){
       normaldata(ii,0) = (*normals)[ii][0];
       normaldata(ii,1) = (*normals)[ii][1];
       normaldata(ii,2) = (*normals)[ii][2];
   }
   return normaldata;
}

Eigen::MatrixXd spMeshVisitor::BoundingBoxVertex(Eigen::MatrixXd vertex, Eigen::VectorXd bounds)
{
    int ctr = 0;
    double xmin = bounds[0], xmax = bounds[1];
    double ymin = bounds[2], ymax = bounds[3];
    double zlim = bounds[4];

    // find number of vertices within boundary used in initialize array
    for(unsigned int ii = 0; ii < vertex.rows(); ii++)
    {
        if(vertex(ii,0) >= xmin && vertex(ii,0) <= xmax && vertex(ii,1) >= ymin && vertex(ii,1) <= ymax && vertex(ii,2) <= zlim)
        {
            ctr++;
        }
    }
    // store selected vertices
    Eigen::MatrixXd newVertices(ctr,3);

    ctr = 0;
    for(unsigned int ii = 0; ii < vertex.rows(); ii++)
    {
        if(vertex(ii,0) >= xmin && vertex(ii,0) <= xmax && vertex(ii,1) >= ymin && vertex(ii,1) <= ymax && vertex(ii,2) <= zlim)
        {
            newVertices(ctr,0) = vertex(ii,0);
            newVertices(ctr,1) = vertex(ii,1);
            newVertices(ctr,2) = vertex(ii,2);
            ctr++;
        }
    }

    return newVertices;
}





