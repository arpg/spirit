#ifndef SP_MESHFUNCTIONS_H
#define SP_MESHFUNCTIONS_H

#include <eigen3/Eigen/Eigen>
#include <spirit/Types/spTypes.h>
#include <numeric>
#include <math.h>

class spMeshFunctions{
public:
    spMeshFunctions();
    ~spMeshFunctions();

    Eigen::MatrixXd ReadFile(const char* file, int filesize);
    Eigen::Vector2d MeshInterpolate(Eigen::VectorXd &pose);
    Eigen::Vector3d MeshDriverPrime(Eigen::MatrixXd *v, Eigen::MatrixXd *n, Eigen::VectorXd &pose, double search_radius, double wheel_radius);
    Eigen::Vector3d MeshDriver(Eigen::Vector3d &normal, Eigen::VectorXd &pose, Eigen::Vector3d &location, double wheel_radius, double height);
    Eigen::Vector3d MeshDriver(Eigen::MatrixXd *v, Eigen::MatrixXd *n, Eigen::VectorXd &pose, double search_radius, double wheel_radius);


    Eigen::VectorXd LocalSearch(Eigen::MatrixXd vertex, double x, double y, double z, double r);
    Eigen::VectorXd MedianAngles(Eigen::MatrixXd normal, Eigen::VectorXd index);
    double CheckPitch(Eigen::VectorXd angles);
    Eigen::VectorXd NearestVertex(Eigen::MatrixXd vertex, Eigen::VectorXd carPose);
    Eigen::VectorXd CheckPoint(Eigen::VectorXd v, Eigen::VectorXd carPose, double rw);

    std::vector<Eigen::MatrixXd*> BB;
    //std::vector<std::vector<double>> BB;

private:

    Eigen::Vector3d nrml, vtx, nx, ny, nz, params;
    std::vector<double> pitch, roll, yaw;
    double min, height, delta_h;
    double med_pitch, med_roll, med_yaw;
    int size, pos;

    // interoplation parameters
    double bx_, by_, mx_, my_;
    Eigen::Vector2d cord; // coordinates for interpolation
    double pitch_, roll_, yaw_;

};

#endif // SP_MESHFUNCTIONS_H
