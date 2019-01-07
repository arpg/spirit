#ifndef CALIBDERCARODE_H__
#define CALIBDERCARODE_H__

#include <eigen3/Eigen/Eigen>
#include <spirit/spGeneralTools.h>
#include <spirit/Types/spTypes.h>
#include <spirit/ODEInterface.h>

class CalibDerODE : public ODEInterface {
public:

  CalibDerODE(){}

  ~CalibDerODE(){}

  const unsigned int StateSpaceSize(){
    return 11;
  }

  const unsigned int ParameterSpaceSize(){
    return 14;
  }

  const unsigned int InputSpaceSize(){
    return 2;
  }

  const bool HasJacobian(){
    return true;
  }

  void Run(Eigen::VectorXd& x_d_t, const Eigen::VectorXd& x_t, const Eigen::VectorXd& u_t){

    // state x_t includes state [x,z0,z1,...,z13]
    if(x_t.size() != (StateSpaceSize()+StateSpaceSize()*ParameterSpaceSize())){
      SPERROREXIT("State Space vector size mismatch! ");
    }
    if(u_t.size() != InputSpaceSize()){
      SPERROREXIT("Input signal vector size mismatch! ");
    }
    if(pp_.size() != ParameterSpaceSize()){
      SPERROREXIT("Parameter vector size mismatch! ");
    }

    double vx_i = x_t[0];
    double vy_i = x_t[1];
    double vw_i = x_t[2];
    double w0_i = x_t[3];
    double w1_i = x_t[4];
    double w2_i = x_t[5];
    double w3_i = x_t[6];
    double chi_i = x_t[9];
    double sigma_i = x_t[10];

    double Ix = pp_[0];
    double Iy = pp_[0];
    double Iw = pp_[2];
    double It = pp_[3];
    double mu_n_p0 = pp_[4];
    double mu_n_p1 = pp_[5];
    double mu_n_p2 = pp_[6];
    double mu_n_p3 = pp_[7];
    double mu_t_p0 = pp_[8];
    double mu_t_p1 = pp_[9];
    double mu_t_p2 = pp_[10];
    double mu_t_p3 = pp_[11];
    double str_del = pp_[12];
    double k_mot = pp_[13];

    double str_in = -u_t[0];
    double Curr_in = u_t[1];

    double r = 0.055;
    double theta = 0.97;
    double d = 0.2029;
    double m=pp_[0];//4.392;//

    // wrapping chi angle

    Eigen::VectorXd x_d(StateSpaceSize()) ;
    /////////////////////////////////////////////////
/*
    double sigma = -u_t[0];
    double Seeffort = u_t[1]*k_mot;

    double wheel_dist = d;

    double TF_c00r = std::sin(sigma);
    double TF_c03r = std::sin(sigma);
    double TF_c04r = std::cos(sigma);
    double TF_c07r = std::cos(sigma);
    double TF_c10r = std::cos(sigma);
    double TF_c13r = std::cos(sigma);
    double TF_c14r = -std::sin(sigma);
    double TF_c17r = -std::sin(sigma);
    double TF_c20r = wheel_dist*std::cos(-sigma+theta);
    double TF_c21r = wheel_dist*std::cos(theta);
    double TF_c22r = -wheel_dist*std::cos(theta);
    double TF_c23r = -wheel_dist*std::cos(theta+sigma);
    double TF_c24r = wheel_dist*std::sin(theta-sigma);
    double TF_c25r = -wheel_dist*std::sin(theta);
    double TF_c26r = -wheel_dist*std::sin(theta);
    double TF_c27r = wheel_dist*std::sin(sigma+theta);

    double Ixi = Ix;
    double Iyi = Iy;
    double Jzi = Iw;
    double Ixstate = x_t[0]*Ixi;
    double Iystate = x_t[1]*Iyi;
    double Jzstate = x_t[2]*Jzi;
    double TireLon0Istate = x_t[3]*It;
    double TireLon1Istate = x_t[4]*It;
    double TireLon2Istate = x_t[5]*It;
    double TireLon3Istate = x_t[6]*It;
    double vx = x_t[0];
    double vy = x_t[1];
    // wrapping chi angle
    double chi_mod = std::fmod(x_t[9]+SP_PI,2*SP_PI);
    if(chi_mod<0){
      chi_mod += 2*SP_PI;
    }
    double chi = chi_mod - SP_PI;

    double beta = std::atan2(vy,vx);

    double GainK = m;
    double TireLon0Ii = It;
    double TireLon1Ii = It;
    double TireLon2Ii = It;
    double TireLon3Ii = It;

    double TireLon0TFr = r;
    double TireLon1TFr = r;
    double TireLon2TFr = r;
    double TireLon3TFr = r;

    double rw0 = r * TireLon0Istate;
    double t0_vlon = vx * cos(sigma);
    double k0 =  (rw0/t0_vlon)-1;
//    double k0 = std::abs(rw0 - t0_vlon)/std::max(std::abs(rw0),std::abs(t0_vlon));
    double rw1 = r * TireLon1Istate;
    double t1_vlon = vx;
    double k1 =  (rw1/t1_vlon)-1;
//    double k1 =  std::abs(rw1 - t1_vlon)/std::max(std::abs(rw1),std::abs(t1_vlon));
    double rw2 = r * TireLon2Istate;
    double t2_vlon = vx;
    double k2 =  (rw2/t2_vlon)-1;
//    double k2 =  std::abs(rw2 - t2_vlon)/std::max(std::abs(rw2),std::abs(t2_vlon));
    double rw3 = r * TireLon3Istate;
    double t3_vlon = vx * cos(sigma);
    double k3 =  (rw3/t3_vlon)-1;
//    double k3 =  std::abs(rw3 - t3_vlon)/std::max(std::abs(rw3),std::abs(t3_vlon));
    double bet0 = (beta - sigma);
    double bet1 = (beta);
    double bet2 = (beta);
    double bet3 = (beta - sigma);

    double TireLon0Rr = mu_n_p0+(mu_n_p1/(mu_n_p2+mu_n_p3*(k0*k0)));
    double TireLon1Rr = mu_n_p0+(mu_n_p1/(mu_n_p2+mu_n_p3*(k1*k1)));
    double TireLon2Rr = mu_n_p0+(mu_n_p1/(mu_n_p2+mu_n_p3*(k2*k2)));
    double TireLon3Rr = mu_n_p0+(mu_n_p1/(mu_n_p2+mu_n_p3*(k3*k3)));

    // lateral friction
    bet0 = sin(bet0);
    bet1 = sin(bet1);
    bet2 = sin(bet2);
    bet3 = sin(bet3);

    double TireLat0Rr = mu_t_p0+(mu_t_p1/(mu_t_p2+mu_t_p3*(bet0*bet0)));
    double TireLat1Rr = mu_t_p0+(mu_t_p1/(mu_t_p2+mu_t_p3*(bet1*bet1)));
    double TireLat2Rr = mu_t_p0+(mu_t_p1/(mu_t_p2+mu_t_p3*(bet2*bet2)));
    double TireLat3Rr = mu_t_p0+(mu_t_p1/(mu_t_p2+mu_t_p3*(bet3*bet3)));

    double TireLon0Ipe = Seeffort - TireLon0TFr * TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r);
    double TireLon1Ipe = Seeffort - TireLon1TFr * TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r);
    double TireLon2Ipe = Seeffort - TireLon2TFr * TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r);
    double TireLon3Ipe = Seeffort - TireLon3TFr * TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r);
    double Ixpe = ((((((TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c10r) + TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r)) + TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r)) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c17r + (-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c14r))) + GainK * (Jzstate / Jzi) * (Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c13r;
    double Iype = ((((-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c04r - TireLat2Rr * ((Jzstate / Jzi) * TF_c26r + Iystate / Iyi)) - TireLat1Rr * ((Jzstate / Jzi) * TF_c25r + Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c03r + TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c00r) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c07r) - GainK * (Jzstate / Jzi) * (Ixstate / Ixi);
    double Jzpe = ((((((((TireLat3Rr * (-((((Jzstate / Jzi) * TF_c27r) + ((Ixstate / Ixi) * TF_c17r)) + ((Iystate / Iyi) * TF_c07r)))) * TF_c27r) + ((TireLon0Rr * ((TireLon0TFr * (TireLon0Istate / TireLon0Ii)) - ((((Iystate / Iyi) * TF_c00r) + ((Ixstate / Ixi) * TF_c10r)) + ((Jzstate / Jzi) * TF_c20r)))) * TF_c20r)) + ((TireLon1Rr * ((TireLon1TFr * (TireLon1Istate / TireLon1Ii)) - ((Ixstate / Ixi) + ((Jzstate / Jzi) * TF_c21r)))) * TF_c21r)) + ((TireLon2Rr * ((TireLon2TFr * (TireLon2Istate / TireLon2Ii)) - ((Ixstate / Ixi) + ((Jzstate / Jzi) * TF_c22r)))) * TF_c22r)) + ((TireLon3Rr * ((TireLon3TFr * (TireLon3Istate / TireLon3Ii)) - ((((Iystate / Iyi) * TF_c03r) + ((Ixstate / Ixi) * TF_c13r)) + ((Jzstate / Jzi) * TF_c23r)))) * TF_c23r)) + ((TireLat0Rr * (-((((Jzstate / Jzi) * TF_c24r) + ((Ixstate / Ixi) * TF_c14r)) + ((Iystate / Iyi) * TF_c04r)))) * TF_c24r)) + ((TireLat1Rr * (-(((Jzstate / Jzi) * TF_c25r) + (Iystate / Iyi)))) * TF_c25r)) + ((TireLat2Rr * (-(((Jzstate / Jzi) * TF_c26r) + (Iystate / Iyi)))) * TF_c26r);

    double V = std::sqrt(std::pow(vx,2)+std::pow(vy,2));

    x_d[0] = Ixpe/Ixi;
    x_d[1] = Iype/Iyi;
    x_d[2] = Jzpe/Jzi;
    x_d[3] = TireLon0Ipe/It;
    x_d[4] = TireLon1Ipe/It;
    x_d[5] = TireLon2Ipe/It;
    x_d[6] = TireLon3Ipe/It;
    // x_dot in inertial frame
    x_d[7] = V*std::cos(chi+beta);
    // x_dot in inertial frame
    x_d[8] = V*std::sin(chi+beta);
    // kai_dot = omega rotational speed
    x_d[9] = Jzstate/Jzi;
*/
    /////////////////////////////////////////////////

    double chi_mod = std::fmod(chi_i+SP_PI,2*SP_PI);
    if(chi_mod<0){
      chi_mod += 2*SP_PI;
    }
    chi_i = chi_mod - SP_PI;

    x_d[0] = (-(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i+d*vw_i*cos(theta))+(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(-vx_i+r*w2_i+d*vw_i*cos(theta))-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(-vy_i*cos(sigma_i)+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta))-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(-r*w0_i+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta))+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i+theta))+m*vw_i*vy_i+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i+theta)))/Ix;
    x_d[1] = -((mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vy_i-d*vw_i*sin(theta))*2.0+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(-r*w0_i+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta))+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i+theta))+m*vw_i*vx_i-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i+theta))-cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(-vy_i*cos(sigma_i)+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta)))/Iy;
    x_d[2] = -(d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i+d*vw_i*cos(theta))+d*sin(sigma_i-theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(-vy_i*cos(sigma_i)+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta))-d*sin(theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vy_i-d*vw_i*sin(theta))*2.0+d*cos(sigma_i-theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(-r*w0_i+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta))+d*sin(sigma_i+theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i+theta))+d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(-vx_i+r*w2_i+d*vw_i*cos(theta))+d*cos(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i+theta)))/Iw;
    x_d[3] = (Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(-r*w0_i+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta)))/It;
    x_d[4] = (Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i+d*vw_i*cos(theta)))/It;
    x_d[5] = (Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(-vx_i+r*w2_i+d*vw_i*cos(theta)))/It;
    x_d[6] = (Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i+theta)))/It;
    x_d[7] = cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
    x_d[8] = sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
    x_d[9] = vw_i;
    x_d[10] = -(sigma_i-str_in)/str_del;


    Eigen::MatrixXd DfDp(StateSpaceSize(),ParameterSpaceSize());
    DfDp = Eigen::MatrixXd::Zero(StateSpaceSize(),ParameterSpaceSize());

    DfDp(0,0) = 1.0/(Ix*Ix)*((mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))-(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*1.0+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))+m*vw_i*vy_i-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0)*-1.0;
    DfDp(0,3) = ((mu_n_p1*mu_n_p3*r*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*-2.0)/vx_i+(mu_n_p1*mu_n_p3*r*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/vx_i+(mu_n_p1*mu_n_p3*r*w0_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i-(mu_n_p1*mu_n_p3*r*w3_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Ix;
    DfDp(0,4) = (vx_i*-2.0-cos(sigma_i)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0+r*w1_i+r*w2_i+cos(sigma_i)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta)))/Ix;
    DfDp(0,5) = ((vx_i*-1.0+r*w1_i*1.0-d*vw_i*cos(theta)*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0))+(vx_i*-1.0+r*w2_i*1.0+d*vw_i*cos(theta)*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0))-(cos(sigma_i)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))+(cos(sigma_i)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))/Ix;
    DfDp(0,6) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))-mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*1.0-mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0+mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)))/Ix;
    DfDp(0,7) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w1_i)/vx_i-1.0,2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))-mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w2_i)/vx_i-1.0,2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*1.0-mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0+mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)))/Ix;
    DfDp(0,8) = (sin(sigma_i)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))-sin(sigma_i)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0)/Ix;
    DfDp(0,9) = ((sin(sigma_i)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*-1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))+(sin(sigma_i)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Ix;
    DfDp(0,10) = (mu_t_p1*sin(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0-mu_t_p1*sin(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0)/Ix;
    DfDp(0,11) = (mu_t_p1*sin(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*-1.0+mu_t_p1*sin(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0)/Ix;
    DfDp(1,1) = 1.0/(Iy*Iy)*((vy_i-d*vw_i*sin(theta)*1.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*2.0+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0-cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+m*vw_i*vx_i+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)));
    DfDp(1,3) = ((mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*-2.0)/vx_i+(mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Iy;
    DfDp(1,4) = (sin(sigma_i)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*-1.0+sin(sigma_i)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)/Iy;
    DfDp(1,5) = ((sin(sigma_i)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*-1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))+(sin(sigma_i)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))/Iy;
    DfDp(1,6) = (mu_n_p1*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*-1.0+mu_n_p1*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0)/Iy;
    DfDp(1,7) = (mu_n_p1*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*-1.0+mu_n_p1*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0)/Iy;
    DfDp(1,8) = (vy_i*-2.0-cos(sigma_i)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0+cos(sigma_i)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+d*vw_i*sin(theta)*2.0)/Iy;
    DfDp(1,9) = ((vy_i*-2.0+d*vw_i*sin(theta)*2.0)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+(cos(sigma_i)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))-(cos(sigma_i)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Iy;
    DfDp(1,10) = (mu_t_p1*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*2.0-mu_t_p1*cos(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+mu_t_p1*cos(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta)))/Iy;
    DfDp(1,11) = (mu_t_p1*cos(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))-mu_t_p1*cos(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+(mu_t_p1*1.0/(vx_i*vx_i)*(vy_i*vy_i)*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Iy;
    DfDp(2,2) = 1.0/(Iw*Iw)*(d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))-d*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*2.0+d*sin(sigma_i+theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))+d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))+d*sin(sigma_i-theta*1.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))+d*cos(sigma_i-theta*1.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))+d*cos(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta)));
    DfDp(2,3) = ((d*mu_n_p1*mu_n_p3*r*w1_i*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/vx_i+(d*mu_n_p1*mu_n_p3*r*w2_i*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/vx_i+(d*mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i-theta*1.0)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i+(d*mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i+theta)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Iw;
    DfDp(2,4) = (d*cos(theta)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*-1.0-d*cos(theta)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*1.0-d*cos(sigma_i-theta*1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0-d*cos(sigma_i+theta)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)/Iw;
    DfDp(2,5) = ((d*cos(sigma_i-theta*1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*-1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))-(d*cos(sigma_i+theta)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0))-(d*cos(theta)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0))-(d*cos(theta)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))/Iw;
    DfDp(2,6) = (d*mu_n_p1*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))+d*mu_n_p1*cos(sigma_i-theta*1.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))+d*mu_n_p1*cos(sigma_i+theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))+d*mu_n_p1*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta)))/Iw;
    DfDp(2,7) = (d*mu_n_p1*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w1_i)/vx_i-1.0,2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))+d*mu_n_p1*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w2_i)/vx_i-1.0,2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))+d*mu_n_p1*cos(sigma_i-theta*1.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))+d*mu_n_p1*cos(sigma_i+theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta)))/Iw;
    DfDp(2,8) = (d*sin(sigma_i+theta)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*-1.0-d*sin(sigma_i-theta*1.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+d*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*2.0)/Iw;
    DfDp(2,9) = ((d*sin(sigma_i-theta*1.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*-1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))+(d*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*2.0)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(d*sin(sigma_i+theta)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Iw;
    DfDp(2,10) = (d*mu_t_p1*sin(sigma_i+theta)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))+d*mu_t_p1*sin(sigma_i-theta*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))-d*mu_t_p1*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*2.0)/Iw;
    DfDp(2,11) = (d*mu_t_p1*sin(sigma_i+theta)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))+d*mu_t_p1*sin(sigma_i-theta*1.0)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))-(d*mu_t_p1*1.0/(vx_i*vx_i)*(vy_i*vy_i)*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Iw;
    DfDp(3,3) = 1.0/(It*It)*(Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)))*-1.0-(mu_n_p1*mu_n_p3*(r*r)*w0_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/(It*vx_i);
    DfDp(3,4) = (r*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)))/It;
    DfDp(3,5) = (r*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0)))/(It*(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)));
    DfDp(3,6) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*-1.0)/It;
    DfDp(3,7) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*-1.0)/It;
    DfDp(3,13) = Curr_in/It;
    DfDp(4,3) = 1.0/(It*It)*(Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta)))*-1.0-(mu_n_p1*mu_n_p3*(r*r)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/(It*vx_i);
    DfDp(4,4) = (r*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta)))/It;
    DfDp(4,5) = (r*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta)))/(It*(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)));
    DfDp(4,6) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*-1.0)/It;
    DfDp(4,7) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w1_i)/vx_i-1.0,2.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*-1.0)/It;
    DfDp(4,13) = Curr_in/It;
    DfDp(5,3) = 1.0/(It*It)*(Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*1.0)*-1.0+(mu_n_p1*mu_n_p3*(r*r)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/(It*vx_i);
    DfDp(5,4) = (r*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*-1.0)/It;
    DfDp(5,5) = (r*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*-1.0)/(It*(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)));
    DfDp(5,6) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta)))/It;
    DfDp(5,7) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*pow((It*r*w2_i)/vx_i-1.0,2.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta)))/It;
    DfDp(5,13) = Curr_in/It;
    DfDp(6,3) = 1.0/(It*It)*(Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0)*-1.0+(mu_n_p1*mu_n_p3*(r*r)*w3_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/(It*vx_i);
    DfDp(6,4) = (r*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*-1.0)/It;
    DfDp(6,5) = (r*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*-1.0)/(It*(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)));
    DfDp(6,6) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta)))/It;
    DfDp(6,7) = (mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta)))/It;
    DfDp(6,13) = Curr_in/It;
    DfDp(10,12) = 1.0/(str_del*str_del)*(sigma_i-str_in*1.0);

    Eigen::MatrixXd DfDx(StateSpaceSize(),StateSpaceSize());
    DfDx = Eigen::MatrixXd::Zero(StateSpaceSize(),StateSpaceSize());

    DfDx(0,0) = (mu_n_p0*-2.0-(mu_n_p1*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0))-(mu_n_p1*1.0)/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0))-pow(cos(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(cos(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(sin(sigma_i),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0+It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0+It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w3_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w0_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0-(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Ix;
    DfDx(0,1) = (m*vw_i+cos(sigma_i)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0+(mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Ix;
    DfDx(0,2) = (m*vy_i-d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*1.0+d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))+d*cos(sigma_i+theta)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))+d*sin(sigma_i+theta)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))-d*cos(sigma_i-theta*1.0)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-d*sin(sigma_i-theta*1.0)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0)/Ix;
    DfDx(0,3) = (r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))+(It*mu_n_p1*mu_n_p3*r*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Ix;
    DfDx(0,4) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))+(It*mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/vx_i)/Ix;
    DfDx(0,5) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))-(It*mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/vx_i)/Ix;
    DfDx(0,6) = (r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))-(It*mu_n_p1*mu_n_p3*r*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Ix;
    DfDx(0,10) = (cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*-1.0+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-d*vw_i*cos(sigma_i+theta)*1.0)*1.0-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*1.0+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0-cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0-mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0+mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0+(It*mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i-(It*mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Ix;
    DfDx(1,0) = (m*vw_i*-1.0+cos(sigma_i)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-mu_t_p1*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i*vx_i)*(vy_i*vy_i)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*2.0-(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)+It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0-It*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/Iy;
    DfDx(1,1) = (mu_t_p0*-2.0-pow(sin(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(sin(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(cos(sigma_i),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-(mu_t_p1*2.0)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+mu_t_p1*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i)*vy_i*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*2.0-(mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+(mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Iy;
    DfDx(1,2) = (m*vx_i*-1.0+d*sin(theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*2.0+d*cos(sigma_i+theta)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-d*sin(sigma_i+theta)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-d*cos(sigma_i-theta*1.0)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0+d*sin(sigma_i-theta*1.0)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0)/Iy;
    DfDx(1,3) = (r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))+(It*mu_n_p1*mu_n_p3*r*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Iy;
    DfDx(1,6) = (r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))-(It*mu_n_p1*mu_n_p3*r*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Iy;
    DfDx(1,10) = (cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-d*vw_i*cos(sigma_i+theta)*1.0)-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0+mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0-mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0+(It*mu_n_p1*mu_n_p3*r*w3_i*pow(sin(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i-(It*mu_n_p1*mu_n_p3*r*w0_i*pow(sin(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Iy;
    DfDx(2,0) = (d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*-1.0+d*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*1.0+d*cos(sigma_i+theta)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0+d*sin(sigma_i+theta)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-d*cos(sigma_i-theta*1.0)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-d*sin(sigma_i-theta*1.0)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0+d*mu_t_p1*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i*vx_i)*(vy_i*vy_i)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*2.0+(d*mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*sin(sigma_i-theta*1.0)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-It*d*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w1_i*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0-It*d*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w2_i*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0+(d*mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*sin(sigma_i+theta)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-It*d*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i-theta*1.0)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0-It*d*mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i+theta)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/Iw;
    DfDx(2,1) = (d*sin(theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*2.0+d*cos(sigma_i+theta)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-d*sin(sigma_i+theta)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-d*cos(sigma_i-theta*1.0)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0+d*sin(sigma_i-theta*1.0)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-d*mu_t_p1*sin(theta)*(vy_i-d*vw_i*sin(theta)*1.0)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i)*vy_i*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*2.0-(d*mu_t_p1*mu_t_p3*sin(sigma_i+theta)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(d*mu_t_p1*mu_t_p3*sin(sigma_i-theta*1.0)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Iw;
    DfDx(2,2) = ((d*d)*pow(sin(sigma_i+theta),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*-1.0-(d*d)*pow(cos(theta),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*1.0-(d*d)*pow(cos(theta),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*1.0-(d*d)*pow(cos(sigma_i-theta*1.0),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-(d*d)*pow(sin(theta),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*2.0-(d*d)*pow(sin(sigma_i-theta*1.0),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-(d*d)*pow(cos(sigma_i+theta),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0)/Iw;
    DfDx(2,3) = (d*r*cos(sigma_i-theta*1.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))+(It*d*mu_n_p1*mu_n_p3*r*cos(sigma_i-theta*1.0)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/Iw;
    DfDx(2,4) = (d*r*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))+(It*d*mu_n_p1*mu_n_p3*r*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/vx_i)/Iw;
    DfDx(2,5) = (d*r*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*-1.0+(It*d*mu_n_p1*mu_n_p3*r*cos(theta)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/vx_i)/Iw;
    DfDx(2,6) = (d*r*cos(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0+(It*d*mu_n_p1*mu_n_p3*r*cos(sigma_i+theta)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Iw;
    DfDx(2,10) = (d*cos(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))-d*sin(sigma_i-theta*1.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*1.0-d*cos(sigma_i+theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*1.0+d*sin(sigma_i+theta)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-d*vw_i*cos(sigma_i+theta)*1.0)+d*cos(sigma_i-theta*1.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))-d*cos(sigma_i-theta*1.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*1.0+d*sin(sigma_i-theta*1.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))+d*sin(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))+d*mu_t_p1*mu_t_p3*sin(sigma_i+theta)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))*2.0+d*mu_t_p1*mu_t_p3*sin(sigma_i-theta*1.0)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*2.0-(It*d*mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i-theta*1.0)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i-(It*d*mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i+theta)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/Iw;
    DfDx(3,0) = (r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))+It*mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/It;
    DfDx(3,1) = (r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))))/It;
    DfDx(3,2) = (d*r*cos(sigma_i-theta*1.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))))/It;
    DfDx(3,3) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0-(It*mu_n_p1*mu_n_p3*(r*r)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/It;
    DfDx(3,10) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)*-1.0+vx_i*sin(sigma_i)+d*vw_i*sin(sigma_i-theta*1.0))*-1.0+(It*mu_n_p1*mu_n_p3*(r*r)*w0_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w0_i*cos(sigma_i))/vx_i-1.0)*(r*w0_i*-1.0+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)+d*vw_i*cos(sigma_i-theta*1.0))*2.0)/vx_i)/It;
    DfDx(4,0) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))+It*mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/It;
    DfDx(4,2) = (d*r*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0))))/It;
    DfDx(4,4) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0)))*-1.0-(It*mu_n_p1*mu_n_p3*(r*r)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w1_i)/vx_i-1.0,2.0),2.0)*((It*r*w1_i)/vx_i-1.0)*(vx_i-r*w1_i*1.0+d*vw_i*cos(theta))*2.0)/vx_i)/It;
    DfDx(5,0) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))-It*mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/It;
    DfDx(5,2) = (d*r*cos(theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*-1.0)/It;
    DfDx(5,5) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0)))*-1.0+(It*mu_n_p1*mu_n_p3*(r*r)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w2_i)/vx_i-1.0,2.0),2.0)*((It*r*w2_i)/vx_i-1.0)*(vx_i*-1.0+r*w2_i+d*vw_i*cos(theta))*2.0)/vx_i)/It;
    DfDx(6,0) = (r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))-It*mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/It;
    DfDx(6,1) = (r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0))))/It;
    DfDx(6,2) = (d*r*cos(sigma_i+theta)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0)/It;
    DfDx(6,6) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0+(It*mu_n_p1*mu_n_p3*(r*r)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/It;
    DfDx(6,10) = (r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0+d*vw_i*sin(sigma_i+theta))-(It*mu_n_p1*mu_n_p3*(r*r)*w3_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((It*r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((It*r*w3_i*cos(sigma_i))/vx_i-1.0)*(r*w3_i-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+d*vw_i*cos(sigma_i+theta))*2.0)/vx_i)/It;
    DfDx(7,0) = vx_i*cos(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)+(1.0/(vx_i*vx_i)*vy_i*sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0);
    DfDx(7,1) = vy_i*cos(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)-(sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*1.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0));
    DfDx(7,9) = sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*-1.0;
    DfDx(8,0) = vx_i*sin(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)-(1.0/(vx_i*vx_i)*vy_i*cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*1.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0);
    DfDx(8,1) = vy_i*sin(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)+(cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i))/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0));
    DfDx(8,9) = cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
    DfDx(9,2) = 1.0;
    DfDx(10,10) = -1.0/str_del;

    x_d_t.segment(0,StateSpaceSize()) = x_d;

    for(int ii=0; ii<ParameterSpaceSize(); ii++){
//      x_d_t.segment((ii+1)*11,11)  = DfDx*x_t.segment((ii+1)*11,11)+ DfDp.block<11,1>(0,ii);
      x_d_t.segment((ii+1)*StateSpaceSize(),StateSpaceSize()) = DfDx*x_t.segment((ii+1)*StateSpaceSize(),StateSpaceSize()) + DfDp.col(ii);
    }
  }

};
#endif //CALIBDERCARODE_H__
