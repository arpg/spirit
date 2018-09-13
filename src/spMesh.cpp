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




Eigen::VectorXd spMesh::localSearch(Eigen::MatrixXd vertex, double x, double y, double z, double r)
{
    // returns index of points that fall within spherical perimeter around car
    Eigen::VectorXd distance(vertex.rows());
    int ctr = 0;

    distance = ( (vertex.col(0).array()-x).array().pow(2)+
                 (vertex.col(1).array()-y).array().pow(2)+
                 (vertex.col(2).array()-z).array().pow(2) ).array().pow(0.5); // */

    // find how many points fall in perimeter
    for(unsigned int ii = 0; ii < distance.rows(); ii++)
    {
        if( distance[ii] < r ) ctr++;
    }

    // store indices
    //std::cout<<"local vertices: "<<ctr<<std::endl;

    Eigen::VectorXd index(ctr);
    ctr = 0;
    for(unsigned int ii = 0; ii < distance.rows(); ii++)
    {
        if( distance[ii] < r )
        {
            index[ctr] = ii;
            ctr++;
        }
    }

    return index;
}

Eigen::VectorXd spMesh::medianAngles(Eigen::MatrixXd normal, Eigen::VectorXd index)
{
    Eigen::VectorXd angles(3);
    std::vector<double> pitch(index.rows());
    std::vector<double> roll(index.rows());
    std::vector<double> yaw(index.rows());
    double med_pitch;
    double med_roll;
    double med_yaw;
    int size;

    // unit x normal
    Eigen::Vector3d nx;
    nx << 1,0,0;

    // unit y normal
    Eigen::Vector3d ny;
    ny << 0,1,0;

    // unit z normal
    Eigen::Vector3d nz;
    nz << 0,0,1;



    // calculate angles
    for(unsigned int ii=0; ii < index.rows(); ii++)
    {
        Eigen::Vector3d n;
        n << normal(index[ii], 0), normal(index[ii], 1), normal(index[ii], 2);

        pitch[ii] = std::atan2(n.cross(nz).norm(), n.dot(nz)); // radians in 4 quadrants
        roll[ii] = std::atan2(n.cross(ny).norm(), n.dot(ny)); // radians in 4 quadrants
        yaw[ii] = std::atan2(n.cross(nx).norm(), n.dot(nx)); // radians in 4 quadrants
    }

    // sort and find median angles
    // pitch

    size = pitch.size();
    if (size == 0 ) med_pitch = 0;
    else
    {
        std::sort(pitch.begin(), pitch.end());
        if( size % 2 == 0) med_pitch = (double)((pitch[size/2 - 1] + pitch[size/2])/2);
        else med_pitch = (double)(pitch[size/2]);
    }

    // roll
    size = roll.size();
    if (size == 0 ) med_roll = 0;
    {
        std::sort(roll.begin(), roll.end());
        if( size % 2 == 0) med_roll = (double)((roll[size/2 - 1] + roll[size/2])/2);
        else med_roll = (double)(roll[size/2]);
    }

    // yaw
    size = yaw.size();
    if (size == 0 ) med_yaw = 0;
    {
    std::sort(yaw.begin(), yaw.end());
    if( size % 2 == 0) med_yaw = (double)((yaw[size/2 - 1] + yaw[size/2])/2);
    else med_yaw = (double)(yaw[size/2]);
    }

    angles<< med_pitch, med_roll, med_yaw;

    return angles;

}

double spMesh::checkPitch(Eigen::VectorXd angles)
{
    double pitch = angles[0];
    const double PI = 3.14;
    if ( pitch > PI/2 ) pitch = PI - pitch;

    // check orientation of normal, ensure car pitches correctly
    if ( angles[2] < PI/2 ) pitch = pitch;
    else if ( angles[2] > PI/2 ) pitch = -1*pitch;

    if ( angles[2] == PI/2)
    {
        if ( angles[1] == 0) pitch = pitch;
        else if ( angles[1] == PI ) pitch = -1*pitch;
    }
    return pitch;
}

Eigen::VectorXd spMesh::nearestVertex(Eigen::MatrixXd vertex, Eigen::VectorXd carPose)
{
    Eigen::VectorXd d(vertex.rows());
    Eigen::VectorXd v(3);
    double x = carPose[0];
    double y = carPose[1];
    double z = carPose[2];

    d = ( (x-vertex.col(0).array()).array().pow(2)+
                 (y-vertex.col(1).array()).array().pow(2)+
                 (z-vertex.col(2).array()).array().pow(2) ).array().pow(0.5);

    int size = d.rows();
    int pos;
    double min = d[0];

    for(int ii=0; ii < size-1 ; ii++)
    {
      if(min > d[ii+1])
      {
         min= d[ii+1]; //compare the first with the others
         pos = ii+1;
      }
    }
    v<< vertex(pos,0), vertex(pos,1), vertex(pos,2);
    return v;
}

Eigen::VectorXd spMesh::checkPoint(Eigen::VectorXd v, Eigen::VectorXd carPose, double rw)
{
    Eigen::VectorXd params(2);

    double distance;
    double x = carPose[0];
    double y = carPose[1];
    double z = carPose[2];
    double dh = .1;

    distance = std::sqrt(std::pow(v[0]-x, 2) + std::pow(v[1]-y, 2) + std::pow(v[2]-z, 2));

    if(z > v[2]) // check if car is above mesh
    {
        if (distance > 2*rw)
        {
            params[0] = 9.81; // apply gravity
            params[1] = (double)(v[2]+dh); // set z-postion of car to height above mesh
        }
    }

    else if(z < v[2])
    {
        params[0] = 0.0;
        params[1] = (double)(v[2]+dh); // set z-postion of car to height above mesh
    }

    else
    {
        params[0] = 0.0;
        params[1] = z;
    }

    return params;
}
















