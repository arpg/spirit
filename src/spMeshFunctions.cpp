#include <spirit/spMeshFunctions.h>
#include <iostream>
#include <vector>
#include <algorithm>

spMeshFunctions::spMeshFunctions(){
    // unit x normal
    nx << 1,0,0;

    // unit y normal
    ny << 0,1,0;

    // unit z normal
    nz << 0,0,1;

    delta_h = 0.1;
    params<< 0,0,0;

    bx_ = 226;
    mx_ = 50;

    by_ = 61;
    my_ = 20;
}

spMeshFunctions::~spMeshFunctions()
{}

Eigen::MatrixXd spMeshFunctions::ReadFile(const char* file, int filesize){

    std::ifstream myfile(file);

    std::string line;

    int row = 0, col = 0, len = 0;

    Eigen::MatrixXd data(filesize + 1, filesize + 1); // can not figure out why I have to add 1 to initialize me confused :(


    if(myfile.is_open()){

       while(getline(myfile, line)){
          //std::cout<<line<< '\n';

          char *ptr = (char *) line.c_str();
          len = line.length();

          //std::cout<<len;

          col = 0;

          char *start = ptr;
          for(int ii = 0; ii < len; ii++){

              if(ptr[ii] == ','){
                  data(row, col++) = atof(start);
                  start = ptr + ii + 1;
              }
          }
          data(row, col) = atof(start);
          row++;

        }
        myfile.close();
    }
    else std::cout<<" File did not open"<<std::endl;

    return data;

}

Eigen::Vector2d spMeshFunctions::MeshInterpolate(Eigen::VectorXd &pose){
    cord[0] = std::round(bx_ + mx_*pose[0]);
    cord[1] = std::round(by_ + my_*pose[1]);

    return cord;
}

Eigen::Vector3d spMeshFunctions::MeshDriver(Eigen::Vector3d &normal, Eigen::VectorXd &pose, Eigen::Vector3d &location,double wheel_radius, double height){
    // determine euler angles
    pitch_ = std::atan2(nrml.cross(nz).norm(), nrml.dot(nz)); // radians in 4 quadrants
    roll_ = std::atan2(nrml.cross(ny).norm(), nrml.dot(ny)); // radians in 4 quadrants
    yaw_ = std::atan2(nrml.cross(nx).norm(), nrml.dot(nx)); // radians in 4 quadrants

    std::cout<<"pitch:  "<<pitch_<<std::endl;

    // check orientation of normal, ensure car pitches correctly
    if ( pitch_ > SP_PI/2 ) pitch_ = SP_PI - pitch_;
    if ( yaw_ < SP_PI/2 ) pitch_ = pitch_;
    else if ( yaw_ > SP_PI/2 ) pitch_ = -1*pitch_;

    if ( yaw_ == SP_PI/2)
    {
        if ( roll_ == 0) pitch_ = pitch_;
        else if ( roll_ == SP_PI ) pitch_ = -1*pitch_;
    }

    params[0] = pitch_;

    // check to see if bike is above mesh
    height =std::sqrt(std::pow(location[0]-pose[0], 2) + std::pow(location[1]-pose[1], 2) + std::pow(location[2]-pose[2], 2));
    if(pose[2] > location[2]) {
        if (height > 2*wheel_radius){
            params[1] = location[2]+delta_h; // set z-postion of car to height above mesh
        }
    }
    else if(pose[2] < location[2]){
        params[1] = location[2]+delta_h; // set z-postion of car to height above mesh
    }

    else{
        params[1] = pose[2];
    }

    return params;
}

Eigen::Vector3d spMeshFunctions::MeshDriverPrime(Eigen::MatrixXd *v, Eigen::MatrixXd *n, Eigen::VectorXd &pose, double search_radius, double wheel_radius){

    // returns index of points that fall within spherical perimeter around car
    Eigen::VectorXd distance((*v).rows());
    distance = ( ((*v).col(0).array()-pose[0]).array().pow(2)+
                 ((*v).col(1).array()-pose[1]).array().pow(2)+
                 ((*v).col(2).array()-pose[2]).array().pow(2) ).array().pow(0.5);

    min = distance[0];
    for(unsigned int ii = 0; ii < distance.rows(); ii++){
        // closest vertex
        if(min > distance[ii]){
            min = distance[ii];
            pos = ii;

        }
        if( distance[ii] < search_radius){
            nrml << (*n)(ii, 0), (*n)(ii, 1), (*n)(ii, 2);
            pitch.push_back(std::atan2(nrml.cross(nz).norm(), nrml.dot(nz))); // radians in 4 quadrants
            roll.push_back(std::atan2(nrml.cross(ny).norm(), nrml.dot(ny))); // radians in 4 quadrants
            yaw.push_back(std::atan2(nrml.cross(nx).norm(), nrml.dot(nx))); // radians in 4 quadrants
        }
    }
    // Closest Vertex
    vtx<< (*v)(pos,0), (*v)(pos,1), (*v)(pos,2);

    // Median Angles
    med_pitch = (double)(std::accumulate(pitch.begin(), pitch.end(), 0)/pitch.size());
    med_roll = (double)(std::accumulate(roll.begin(), roll.end(), 0)/roll.size());
    med_yaw = (double)(std::accumulate(yaw.begin(), yaw.end(), 0)/yaw.size());

    //std::cout<<"size:   "<<size<<std::endl;
    //std::cout<<"pitch size:   "<<pitch.size()<<std::endl;

    //std::cout<<vtx[0]<<" "<<vtx[1]<<" "<<vtx[2]<<std::endl;
    //std::cout<<pos<<std::endl;

    //std::cout<<"pitch:  "<<med_pitch<<std::endl;

    // check orientation of normal, ensure car pitches correctly
    if ( med_pitch > SP_PI/2 ) med_pitch = SP_PI - med_pitch;
    if ( med_yaw < SP_PI/2 ) med_pitch = med_pitch;
    else if ( med_yaw > SP_PI/2 ) med_pitch = -1*med_pitch;

    if ( med_yaw == SP_PI/2)
    {
        if ( med_roll == 0) med_pitch = med_pitch;
        else if ( med_roll == SP_PI ) med_pitch = -1*med_pitch;
    }

    params[0] = med_pitch;

    // check to see if bike is above mesh
    height =std::sqrt(std::pow(vtx[0]-pose[0], 2) + std::pow(vtx[1]-pose[1], 2) + std::pow(vtx[2]-pose[2], 2));
    if(pose[2] > vtx[2]) {
        if (height > 2*wheel_radius){
            params[1] = vtx[2]+delta_h; // set z-postion of car to height above mesh
        }
    }
    else if(pose[2] < vtx[2]){
        params[1] = vtx[2]+delta_h; // set z-postion of car to height above mesh
    }

    else{
        params[1] = pose[2];
    }

    return params;
}

Eigen::Vector3d spMeshFunctions::MeshDriver(Eigen::MatrixXd *v, Eigen::MatrixXd *n, Eigen::VectorXd &pose, double search_radius, double wheel_radius){

    // returns index of points that fall within spherical perimeter around car
    Eigen::VectorXd distance((*v).rows());
    distance = ( ((*v).col(0).array()-pose[0]).array().pow(2)+
                 ((*v).col(1).array()-pose[1]).array().pow(2)+
                 ((*v).col(2).array()-pose[2]).array().pow(2) ).array().pow(0.5);

    min = distance[0];
    for(unsigned int ii = 0; ii < distance.rows(); ii++){
        // closest vertex
        if(min > distance[ii]){
            min = distance[ii];
            pos = ii;

        }
        if( distance[ii] < search_radius){
            nrml << (*n)(ii, 0), (*n)(ii, 1), (*n)(ii, 2);
            pitch.push_back(std::atan2(nrml.cross(nz).norm(), nrml.dot(nz))); // radians in 4 quadrants
            roll.push_back(std::atan2(nrml.cross(ny).norm(), nrml.dot(ny))); // radians in 4 quadrants
            yaw.push_back(std::atan2(nrml.cross(nx).norm(), nrml.dot(nx))); // radians in 4 quadrants
        }
    }

    size = pitch.size();

    //std::cout<<"size:   "<<size<<std::endl;
    //std::cout<<"pitch size:   "<<pitch.size()<<std::endl;

    vtx<< (*v)(pos,0), (*v)(pos,1), (*v)(pos,2);
    //std::cout<<vtx[0]<<" "<<vtx[1]<<" "<<vtx[2]<<std::endl;
    //std::cout<<pos<<std::endl;

    // sort and find median angles

    if (size == 0){
        med_pitch = 0;
        med_roll = 0;
        med_yaw = 0;
    }
    else{
        std::sort(pitch.begin(), pitch.end());
        std::sort(roll.begin(), roll.end());
        std::sort(yaw.begin(), yaw.end());

        if( size % 2 == 0){
            med_pitch = (double)((pitch[size/2 - 1] + pitch[size/2])/2);
            med_roll = (roll[size/2 - 1] + roll[size/2])/2;
            med_yaw = (yaw[size/2 - 1] + yaw[size/2])/2;
        }
        else{
            med_pitch = (double)(pitch[size/2]);
            med_roll = roll[size/2];
            med_yaw = yaw[size/2];
        }
    }
    //std::cout<<"pitch:  "<<med_pitch<<std::endl;

    // check orientation of normal, ensure car pitches correctly
    if ( med_pitch > SP_PI/2 ) med_pitch = SP_PI - med_pitch;
    if ( med_yaw < SP_PI/2 ) med_pitch = med_pitch;
    else if ( med_yaw > SP_PI/2 ) med_pitch = -1*med_pitch;

    if ( med_yaw == SP_PI/2)
    {
        if ( med_roll == 0) med_pitch = med_pitch;
        else if ( med_roll == SP_PI ) med_pitch = -1*med_pitch;
    }

    params[0] = med_pitch;

    // check to see if bike is above mesh
    height =std::sqrt(std::pow(vtx[0]-pose[0], 2) + std::pow(vtx[1]-pose[1], 2) + std::pow(vtx[2]-pose[2], 2));
    if(pose[2] > vtx[2]) {
        if (height > 2*wheel_radius){
            params[1] = vtx[2]+delta_h; // set z-postion of car to height above mesh
        }
    }
    else if(pose[2] < vtx[2]){
        params[1] = vtx[2]+delta_h; // set z-postion of car to height above mesh
    }

    else{
        params[1] = pose[2];
    }

    return params;
}

Eigen::VectorXd spMeshFunctions::LocalSearch(Eigen::MatrixXd vertex, double x, double y, double z, double r)
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

Eigen::VectorXd spMeshFunctions::MedianAngles(Eigen::MatrixXd normal, Eigen::VectorXd index)
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

double spMeshFunctions::CheckPitch(Eigen::VectorXd angles)
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

Eigen::VectorXd spMeshFunctions::NearestVertex(Eigen::MatrixXd vertex, Eigen::VectorXd carPose)
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

Eigen::VectorXd spMeshFunctions::CheckPoint(Eigen::VectorXd v, Eigen::VectorXd carPose, double rw)
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
