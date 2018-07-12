SET( spirit_LIBRARIES  "/usr/lib/x86_64-linux-gnu/libGLU.so;/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLEW.so;/usr/lib/x86_64-linux-gnu/libglut.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so;/usr/lib/x86_64-linux-gnu/libassimp.so;/usr/lib/x86_64-linux-gnu/libIL.so;/usr/lib/x86_64-linux-gnu/libILU.so;/usr/lib/x86_64-linux-gnu/libILUT.so;/home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so;/usr/local/lib/libBulletDynamics.a;/usr/local/lib/libBulletCollision.a;/usr/local/lib/libLinearMath.a;/usr/local/lib/libBulletSoftBody.a;/usr/local/lib/libceres.so;/usr/lib/x86_64-linux-gnu/libglog.so;/usr/lib/x86_64-linux-gnu/libcholmod.so;/usr/lib/x86_64-linux-gnu/libccolamd.so;/usr/lib/x86_64-linux-gnu/libcamd.so;/usr/lib/x86_64-linux-gnu/libcolamd.so;/usr/lib/x86_64-linux-gnu/libamd.so;/usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so;/usr/lib/liblapack.so;/usr/lib/libblas.so;/usr/lib/x86_64-linux-gnu/libcxsparse.so;gomp;/usr/lib/x86_64-linux-gnu/libprotobuf.so;/usr/local/lib/libosg.so;/usr/local/lib/libosgViewer.so;/usr/local/lib/libosgUtil.so;/usr/local/lib/libosgDB.so;/usr/local/lib/libosgGA.so;/usr/local/lib/libOpenThreads.so;/home/boston/Documents/spirit_dep/spirit/build/libspirit.so" CACHE INTERNAL "spirit libraries" FORCE )
SET( spirit_INCLUDE_DIRS  /home/boston/Documents/spirit_dep/spirit/build;/usr/include;/usr/include/x86_64-linux-gnu;/usr/include/eigen3;/home/boston/Documents/spirit_dep/Pangolin/include;/home/boston/Documents/spirit_dep/Pangolin/build/include;/usr/include/IL;/usr/include/IL/..;/home/boston/Documents/spirit_dep/SceneGraph/include;/home/boston/Documents/spirit_dep/SceneGraph/build/include;/usr/local/include/bullet;/usr/local/include;/home/boston/Documents/spirit_dep/spirit/include;/home/boston/Documents/spirit_dep/spirit/include;/home/boston/Documents/spirit_dep/spirit/build/include CACHE INTERNAL "spirit include directories" FORCE )
SET( spirit_LIBRARY_DIRS  CACHE INTERNAL "spirit library directories" FORCE )

mark_as_advanced( spirit_LIBRARIES )
mark_as_advanced( spirit_LIBRARY_DIRS )
mark_as_advanced( spirit_INCLUDE_DIRS )



# Compute paths
get_filename_component( PACKAGE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

# This file, when used for INSTALLED code, does not use Targets... sigh.
## Library dependencies (contains definitions for IMPORTED targets)
#if(NOT TARGET "spirit_LIBRARIES" AND NOT "spirit_BINARY_DIR")
#    include( "${PACKAGE_CMAKE_DIR}/spiritTargets.cmake" )
#    include( "${PACKAGE_CMAKE_DIR}/spiritConfigVersion.cmake" )
#endif()

#SET(spirit_LIBRARIES )
#SET(spirit_LIBRARY )
#SET(spirit_INCLUDE_DIRS /home/boston/Documents/spirit_dep/spirit/build;/usr/include;/usr/include/x86_64-linux-gnu;/usr/include/eigen3;/home/boston/Documents/spirit_dep/Pangolin/include;/home/boston/Documents/spirit_dep/Pangolin/build/include;/usr/include/IL;/usr/include/IL/..;/home/boston/Documents/spirit_dep/SceneGraph/include;/home/boston/Documents/spirit_dep/SceneGraph/build/include;/usr/local/include/bullet;/usr/local/include;/home/boston/Documents/spirit_dep/spirit/include;/home/boston/Documents/spirit_dep/spirit/include;/home/boston/Documents/spirit_dep/spirit/build/include)
#SET(spirit_LINK_DIRS )
