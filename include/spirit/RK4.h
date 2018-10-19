#ifndef RK4_H__
#define RK4_H__

#include <functional>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <spirit/spGeneralTools.h>
#include <spirit/Types/spTypes.h>

template<typename ODEInterface>
class RK4 {
public:

  RK4(double step_size){
    int_step_ = step_size;
    // SINA: hardcoded extended_state size
    extended_state_.resize(11*15);
    extended_state_ = Eigen::VectorXd::Zero(extended_state_.size());
  }
  ~RK4(){}

  void SetBaseTimestamp(double base_time){
    base_time_ = base_time;
  }

//  void SetJacobianMatrix(Eigen::MatrixXd& jac_mat){
//    if((jac_mat.rows() != 0)||(jac_mat.cols() != 0)){
//       SPERROREXIT("Jacobian should have size zero when initialized");
//    }
//    jacobian_ = jac_mat;
//    has_jacobian_ = true;
//  }

  Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, double horizon){
    Eigen::VectorXd dummy_u;
    Eigen::MatrixXd dummy_jac;
    Eigen::MatrixXd trajectory;
    Solve(curr_state,dummy_u,horizon,trajectory,dummy_jac);
    return trajectory;
  }

  Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon){
    Eigen::MatrixXd dummy_jac;
    Eigen::MatrixXd trajectory;
    Solve(curr_state,u_t,horizon,trajectory,dummy_jac);
    return trajectory;
  }


  void SolveOnce(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double step_size,  Eigen::MatrixXd& jacobian){
    if(target_ode_.HasJacobian()){
      if(jacobian.size() == 0){
        SPERROREXIT("A Jacobian matrix was expected but not provided in function call RK4::SolveOnce");
      }
      if((jacobian.rows() != curr_state.size()) || (jacobian.cols() != target_ode_.ParameterSpaceSize())){
        SPERROREXIT("Jacobian size does not match");
      }
    }
    extended_state_.head(curr_state.size()) = curr_state;
    int_step_ = step_size;
    RK4Iteration(extended_state_,u_t);
    for(int jj=0; jj<target_ode_.ParameterSpaceSize(); jj++){
      jacobian.col(jj) = extended_state_.segment((jj+1)*target_ode_.StateSpaceSize(),target_ode_.StateSpaceSize());
    }

    curr_state = extended_state_.head(curr_state.size());
  }

  void Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon,Eigen::MatrixXd& trajectory,  Eigen::MatrixXd& jacobian){
//    if((jacobian != nullptr)&&(!target_ode_.HasJacobian())){
//      SPERROREXIT("Templated class doesn't have a jacobian but one has been Introduced here.");
//    }

    int num_samples = (int)(horizon/int_step_);
    trajectory.resize(curr_state.rows(),num_samples);
    Eigen::VectorXd extended_state;
//    full_jac_.resize(curr_state.size()*num_samples,14);
    if(target_ode_.HasJacobian()){
      if(jacobian.size() == 0){
        SPERROREXIT("A Jacobian matrix was expected but not provided in function call RK4::Solve");
      }
      if((jacobian.rows() != curr_state.size()*num_samples) || (jacobian.cols() != target_ode_.ParameterSpaceSize())){
        SPERROREXIT("Jacobian size does not match");
      }
      // SINA: hard coded parameter dimentions here
      extended_state.resize(curr_state.size()*15);
      // Initialize z states to zero
      extended_state = Eigen::VectorXd::Zero(extended_state.size());
    }
    extended_state.head(curr_state.size()) = curr_state;
    for(int ii=0; ii<num_samples; ii++) {
      RK4Iteration(extended_state,u_t);
      if(target_ode_.HasJacobian()){
        for(int jj=0; jj<target_ode_.ParameterSpaceSize(); jj++){
//          full_jac_.block<target_ode_.StateSpaceSize(),1>(ii*target_ode_.StateSpaceSize(),jj) = extended_state.segment((jj+1)*target_ode_.StateSpaceSize(),target_ode_.StateSpaceSize());
          jacobian.col(jj).segment(ii*target_ode_.StateSpaceSize(),target_ode_.StateSpaceSize()) = extended_state.segment((jj+1)*target_ode_.StateSpaceSize(),target_ode_.StateSpaceSize());
        }
//        std::cout << "jac is " << jacobian(4+(ii*11),0) << std::endl;
      }

      trajectory.col(ii) = extended_state.head(curr_state.size());
    }

    curr_state = extended_state.head(curr_state.size());
  }

  void SetParameterVec(const Eigen::VectorXd& pp){
    target_ode_.SetParameterVec(pp);
  }

private:
  void RK4Iteration(Eigen::VectorXd& x_t,const Eigen::VectorXd& u_t){
    // actual RK4 step
    Eigen::VectorXd k1(x_t.size());
    Eigen::VectorXd k2(x_t.size());
    Eigen::VectorXd k3(x_t.size());
    Eigen::VectorXd k4(x_t.size());
    target_ode_.Run(k1,x_t,u_t);
    target_ode_.Run(k2,x_t+int_step_*(0.5*k1),u_t);
    target_ode_.Run(k3,x_t+int_step_*(0.5*k2),u_t);
    target_ode_.Run(k4,x_t+int_step_*k3,u_t);
    x_t = x_t+(int_step_/6.0)*(k1+2*k2+2*k3+k4);

    // Sina: this is a hack, move rotation angle wrap around to somewhere appropriate
    double chi_mod = std::fmod(x_t[9]+SP_PI,2*SP_PI);
    if(chi_mod<0){
      chi_mod += 2*SP_PI;
    }
    x_t[9] = chi_mod - SP_PI;
  }

  double int_step_;
  ODEInterface target_ode_;
  double base_time_;
  Eigen::VectorXd extended_state_;

};

#endif    //RK4_H__
