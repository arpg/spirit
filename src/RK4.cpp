#include <spirit/RK4.h>

template<typename ODE_TYPE>
RK4<ODE_TYPE>::RK4(double step_size)


template<typename ODE_TYPE>
RK4<ODE_TYPE>::~RK4(){
}

// Solve Atunomous Diff Eq
//void RK4::RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd)> ode_func) {
//  ode_auto_func_ = ode_func;
//}

template<typename ODE_TYPE>
Eigen::MatrixXd RK4<ODE_TYPE>::Solve(Eigen::VectorXd& curr_state, double horizon)

template<typename ODE_TYPE>
void RK4<ODE_TYPE>::RK4Iteration(Eigen::VectorXd& x_t)

// Solve Diff Eq with system inputs
//void RK4::RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd,const Eigen::VectorXd)> ode_func) {
//template<typename ODE_F>
//void RK4::RegisterODE(ODE_F ode_func){
//  : ode_func_(std::bind(ode_func,std::placeholders::_1,std::placeholders::_2)){

//  ode_func_ = std::bind(ode_func,std::placeholders::_1,std::placeholders::_2);
//  ode_func_ = ode_func;
//}

template<typename ODE_TYPE>
Eigen::MatrixXd RK4<ODE_TYPE>::Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon)

template<typename ODE_TYPE>
void RK4<ODE_TYPE>::RK4Iteration(Eigen::VectorXd& x_t, const Eigen::VectorXd& u_t)
