#ifndef ANALYTICALCALIBCOSTFUNC_H__
#define ANALYTICALCALIBCOSTFUNC_H__

#include <ceres/ceres.h>
#include "LogParser.h"
#include <spirit/spirit.h>
#include <spirit/CalibDerCarODE.h>
#include <fstream>
#include <iostream>

class AnalyticalCalibCostFunc : public ceres::DynamicCostFunction {
 public:
  AnalyticalCalibCostFunc(const LogParser& log, unsigned int num_params) : log_(log) {
    SetNumResiduals(log.GetNumMeaseurements());
    AddParameterBlock(num_params);
    std::cout << "residual size is " << num_residuals() << std::endl;
  }

  ~AnalyticalCalibCostFunc() {}
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<const Eigen::VectorXd> params(parameters[0],parameter_block_sizes()[0]);
    if ((jacobians != NULL) && (residuals != NULL)) {
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> jac(jacobians[0],num_residuals(),parameter_block_sizes()[0]);
      Eigen::Map<Eigen::VectorXd> res(residuals,num_residuals());

      // create a RK4 solver object
      RK4<CalibDerODE> rk4solver(0.01);

      rk4solver.SetParameterVec(params);

      Eigen::MatrixXd iter_jac(11,14);
      Eigen::VectorXd curr_u(2);
      Eigen::VectorXd curr_state(11);
      curr_state = Eigen::VectorXd::Zero(11);
      if(log_.state_input_vec_[1].data_type == 1){
        curr_state[3] = log_.state_input_vec_[1].state[0];
        curr_state[4] = log_.state_input_vec_[1].state[1];
        curr_state[5] = log_.state_input_vec_[1].state[2];
        curr_state[6] = log_.state_input_vec_[1].state[3];
        curr_state[10] = log_.state_input_vec_[1].state[4];
      }
      if(log_.state_input_vec_[0].data_type == 2){
        curr_state[7] = log_.state_input_vec_[0].state[0];
        curr_state[8] = log_.state_input_vec_[0].state[1];
        curr_state[9] = log_.state_input_vec_[0].state[2];
        curr_state[0] = log_.state_input_vec_[0].state[3];
        curr_state[1] = log_.state_input_vec_[0].state[4];
        curr_state[2] = log_.state_input_vec_[0].state[5];
      }

      if(curr_state[0] == 0){
        curr_state[0] = 0.01;
      }

      double time_diff;
      int row_count = 0;

      std::ofstream myfile;
      myfile.open("path.csv",std::ofstream::trunc);

//      std::ofstream myfile2;
//      myfile2.open("path2.csv",std::ofstream::trunc);


      for(unsigned long int ii=0; ii<log_.state_input_vec_.size()-1; ii++){
        curr_u[0] = log_.state_input_vec_[ii].input[0];
        curr_u[1] = log_.state_input_vec_[ii].input[1];
        time_diff = log_.state_input_vec_[ii+1].timestamp - log_.state_input_vec_[ii].timestamp;
        rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

        double linvel_x = std::cos(curr_state[9])*curr_state[0] - std::sin(curr_state[9])*curr_state[1];
        double linvel_y = std::sin(curr_state[9])*curr_state[0] + std::cos(curr_state[9])*curr_state[1];

        for(int ii=0; ii<11; ii++){
          if(ii==0){
            myfile << linvel_x << ",";
          } else if(ii==1){
            myfile << linvel_y << ",";
          } else {
            myfile << curr_state[ii] << ",";
          }
        }
        myfile << "\n";

        if(log_.state_input_vec_[ii+1].data_type == 1){

          jac.row(row_count) = iter_jac.row(3);
          res[row_count] = curr_state[3] - log_.state_input_vec_[ii+1].state[0];
          res[row_count] *= 0.1;
          row_count++;
          jac.row(row_count) = iter_jac.row(4);
          res[row_count] = curr_state[4] - log_.state_input_vec_[ii+1].state[1];
          res[row_count] *= 0.1;
          row_count++;
          jac.row(row_count) = iter_jac.row(5);
          res[row_count] = curr_state[5] - log_.state_input_vec_[ii+1].state[2];
          res[row_count] *= 0.1;
          row_count++;
          jac.row(row_count) = iter_jac.row(6);
          res[row_count] = curr_state[6] - log_.state_input_vec_[ii+1].state[3];
          res[row_count] *= 0.1;
          row_count++;
          jac.row(row_count) = iter_jac.row(10);
          res[row_count] = curr_state[10] - log_.state_input_vec_[ii+1].state[4];
          row_count++;

//          for(int jj=0; jj<11; jj++){
//            if((jj<=6)&&(jj>=3)){
//              myfile2 << log_.state_input_vec_[ii+1].state[jj-3] << ",";
//            } else if (jj==10) {
//              myfile2 << log_.state_input_vec_[ii+1].state[jj-6] << ",";
//            } else {
//              myfile2 << 0 << ",";
//            }
//          }
//          myfile2 << "\n";
        }

        if(log_.state_input_vec_[ii+1].data_type == 2){
//          Eigen::VectorXd zer(14);
//          zer = Eigen::VectorXd::Zero(14);

          jac.row(row_count) = iter_jac.row(7);
          res[row_count] = curr_state[7] - log_.state_input_vec_[ii+1].state[0];
          res[row_count] *= 10;
          row_count++;
          jac.row(row_count) = iter_jac.row(8);
          res[row_count] = curr_state[8] - log_.state_input_vec_[ii+1].state[1];
          res[row_count] *= 10;
          row_count++;
          jac.row(row_count) = iter_jac.row(9);
          res[row_count] = curr_state[9] - log_.state_input_vec_[ii+1].state[2];
          res[row_count] *= 0.1;
          row_count++;

          jac.row(row_count) = iter_jac.row(0);
          res[row_count] = curr_state[0] - log_.state_input_vec_[ii+1].state[3];
          row_count++;
          jac.row(row_count) = iter_jac.row(1);
          res[row_count] = curr_state[1] - log_.state_input_vec_[ii+1].state[4];
          row_count++;
          jac.row(row_count) = iter_jac.row(2);
          res[row_count] = curr_state[2] - log_.state_input_vec_[ii+1].state[5];
          row_count++;

//          double chi = log_.state_input_vec_[ii+1].state[2];
//          double linx = std::cos(chi)*log_.state_input_vec_[ii+1].state[3] - std::sin(chi)*log_.state_input_vec_[ii+1].state[4];
//          double liny = std::sin(chi)*log_.state_input_vec_[ii+1].state[3] + std::cos(chi)*log_.state_input_vec_[ii+1].state[4];
//          for(int jj=0; jj<11; jj++){
//            if(jj==0){
//              myfile2 << linx << ",";
//            } else if(jj==1){
//              myfile2 << liny << ",";
//            } else if(jj==2){
//                myfile2 << log_.state_input_vec_[ii+1].state[5] << ",";
//            } else if ((jj>=7)&&(jj<=9)) {
//              myfile2 << log_.state_input_vec_[ii+1].state[jj-7] << ",";
//            } else {
//              myfile2 << 0 << ",";
//            }
//          }
//          myfile2 << "\n";

        }
      }

      myfile.close();
//      myfile2.close();

//      std::cout << " jac is \n" << jac << std::endl;
//      std::cout << "res is " << res << std::endl;
//      std::cout << "jac size " << jac.rows() << " - " << jac.cols() << std::endl;
//      std::cout << "res size " << res.size() << std::endl;
//      SPERROREXIT ("done here");

    } else if (residuals != NULL) {
      Eigen::Map<Eigen::VectorXd> res(residuals,num_residuals());
      std::cout << "doing residuals" << std::endl;
      // create a RK4 solver object
      RK4<CalibDerODE> rk4solver(0.01);

      rk4solver.SetParameterVec(params);

      Eigen::MatrixXd iter_jac(11,14);
      Eigen::VectorXd curr_u(2);
      Eigen::VectorXd curr_state(11);
      curr_state = Eigen::VectorXd::Zero(11);
      if(log_.state_input_vec_[1].data_type == 1){
        curr_state[3] = log_.state_input_vec_[1].state[0];
        curr_state[4] = log_.state_input_vec_[1].state[1];
        curr_state[5] = log_.state_input_vec_[1].state[2];
        curr_state[6] = log_.state_input_vec_[1].state[3];
        curr_state[10] = log_.state_input_vec_[1].state[4];
      }
      if(log_.state_input_vec_[0].data_type == 2){
        curr_state[7] = log_.state_input_vec_[0].state[0];
        curr_state[8] = log_.state_input_vec_[0].state[1];
        curr_state[9] = log_.state_input_vec_[0].state[2];
        curr_state[0] = log_.state_input_vec_[0].state[3];
        curr_state[1] = log_.state_input_vec_[0].state[4];
        curr_state[2] = log_.state_input_vec_[0].state[5];
      }

      if(curr_state[0] == 0){
        curr_state[0] = 0.01;
      }

      double time_diff;
      int row_count = 0;

      for(unsigned long int ii=0; ii<log_.state_input_vec_.size()-1; ii++){
        curr_u[0] = log_.state_input_vec_[ii].input[0];
        curr_u[1] = log_.state_input_vec_[ii].input[1];
        time_diff = log_.state_input_vec_[ii+1].timestamp - log_.state_input_vec_[ii].timestamp;
        rk4solver.SolveOnce(curr_state,curr_u,time_diff,iter_jac);

        if(log_.state_input_vec_[ii+1].data_type == 1){
          res[row_count] = curr_state[3] - log_.state_input_vec_[ii+1].state[0];
          res[row_count] *= 0.1;
          row_count++;
          res[row_count] = curr_state[4] - log_.state_input_vec_[ii+1].state[1];
          res[row_count] *= 0.1;
          row_count++;
          res[row_count] = curr_state[5] - log_.state_input_vec_[ii+1].state[2];
          res[row_count] *= 0.1;
          row_count++;
          res[row_count] = curr_state[6] - log_.state_input_vec_[ii+1].state[3];
          res[row_count] *= 0.1;
          row_count++;
          res[row_count] = curr_state[10] - log_.state_input_vec_[ii+1].state[4];
          row_count++;
        }

        if(log_.state_input_vec_[ii+1].data_type == 2){
          res[row_count] = curr_state[7] - log_.state_input_vec_[ii+1].state[0];
          res[row_count] *= 10;
          row_count++;
          res[row_count] = curr_state[8] - log_.state_input_vec_[ii+1].state[1];
          res[row_count] *= 10;
          row_count++;
          res[row_count] = curr_state[9] - log_.state_input_vec_[ii+1].state[2];
          res[row_count] *= 0.1;
          row_count++;

          res[row_count] = curr_state[0] - log_.state_input_vec_[ii+1].state[3];
          row_count++;
          res[row_count] = curr_state[1] - log_.state_input_vec_[ii+1].state[4];
          row_count++;
          res[row_count] = curr_state[2] - log_.state_input_vec_[ii+1].state[5];
          row_count++;

        }
      }
//      std::cout << "res \n" << res << std::endl;
    }

    return true;
  }

private:
  const LogParser& log_;
};

#endif //ANALYTICALCALIBCOSTFUNC_H__
