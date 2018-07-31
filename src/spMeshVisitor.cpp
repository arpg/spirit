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

    for ( unsigned int i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Drawable* drawable = geode.getDrawable(i);
        //std::cout << spaces() << drawable->libraryName() << "::" << drawable->className() << std::endl;
    }

    traverse( geode );
    _level--;
}
