#ifndef BIKEODE_H__
#define BIKEODE_H__

#include <eigen3/Eigen/Eigen>

inline Eigen::ArrayXd BikeODE(Eigen::VectorXd y_t, const Eigen::VectorXd u_t){

    const double lr = 0.21;
    const double lf = 0.21;

    double sf = u_t[0]; // steering control input
    float beta = std::atan2(lr*std::tan(sf), lr+lf);
    double accel = u_t[1]; // acceleration control input
    double pitch = u_t[2];

    Eigen::VectorXd y_dot(y_t);
    double phi = y_t[3];
    double V = y_t[4];

    y_dot[0] = V*std::cos(phi + beta); // x global
    y_dot[1] = V*std::sin(phi + beta); // y global
    y_dot[2] = V*std::sin(pitch); // - g*dt;
    y_dot[3] = V/lr*std::sin(beta); // phi is yaw
    y_dot[4] = accel;

    //std::cout<< "The pitch is: " << pitch*57.2958 << " degrees" << std::endl;
    return y_dot;
}

#endif //BIKEODE_H__
