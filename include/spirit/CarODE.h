#ifndef CARODE_H__
#define CARODE_H__

#include <eigen3/Eigen/Eigen>

inline Eigen::ArrayXd CarODE(const Eigen::VectorXd y_t, const Eigen::VectorXd u_t) {
  double sigma = -u_t[0];
  double Seeffort = u_t[1];
  double theta = SP_PI/4.0;

  double wheel_dist = 0.2;

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

  double Ixi = 2;
  double Iyi = 2;
  double Jzi = 2;
  double Ixstate = y_t[0]*Ixi;
  double Iystate = y_t[1]*Iyi;
  double Jzstate = y_t[2]*Jzi;
  double TireLon0Istate = y_t[3];
  double TireLon1Istate = y_t[4];
  double TireLon2Istate = y_t[5];
  double TireLon3Istate = y_t[6];
  double vx = y_t[0];
  double vy = y_t[1];
  // wrapping chi angle
  double chi_mod = std::fmod(y_t[9]+SP_PI,2*SP_PI);
  if(chi_mod<0){
    chi_mod += 2*SP_PI;
  }
  double chi = chi_mod - SP_PI;

  double beta = std::atan2(vy,vx);

  double GainK = 12;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;

  double Radius = 0.1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;

  double rw0 = Radius * TireLon0Istate;
  double t0_vlon = vx * cos(sigma);
  double k0 =  std::abs(rw0 - t0_vlon)/std::max(std::abs(rw0),std::abs(t0_vlon));
  double rw1 = Radius * TireLon1Istate;
  double t1_vlon = vx;
  double k1 =  std::abs(rw1 - t1_vlon)/std::max(std::abs(rw1),std::abs(t1_vlon));
  double rw2 = Radius * TireLon2Istate;
  double t2_vlon = vx;
  double k2 =  std::abs(rw2 - t2_vlon)/std::max(std::abs(rw2),std::abs(t2_vlon));
  double rw3 = Radius * TireLon3Istate;
  double t3_vlon = vx * cos(sigma);
  double k3 =  std::abs(rw3 - t3_vlon)/std::max(std::abs(rw3),std::abs(t3_vlon));
  double bet0 = std::abs(beta - sigma);
  double bet1 = std::abs(beta);
  double bet2 = std::abs(beta);
  double bet3 = std::abs(beta - sigma);


  double TireLon0Rr = 2.2+(1/(1+20*(k0*k0)));
  double TireLon1Rr = 2.2+(1/(1+20*(k1*k1)));
  double TireLon2Rr = 2.2+(1/(1+20*(k2*k2)));
  double TireLon3Rr = 2.2+(1/(1+20*(k3*k3)));

  // lateral friction
  bet0 = sin(bet0);
  bet1 = sin(bet1);
  bet2 = sin(bet2);
  bet3 = sin(bet3);

  double TireLat0Rr = 1.5+(0.2/(1+20*(bet0*bet0)));
  double TireLat1Rr = 1.5+(0.2/(1+20*(bet1*bet1)));
  double TireLat2Rr = 1.5+(0.2/(1+20*(bet2*bet2)));
  double TireLat3Rr = 1.5+(0.2/(1+20*(bet3*bet3)));

//  double TireLat0Rr = -(bet0*bet0)+10;
//  double TireLat1Rr = -(bet1*bet1)+10;
//  double TireLat2Rr = -(bet2*bet2)+10;
//  double TireLat3Rr = -(bet3*bet3)+10;

  double TireLon0Ipe = Seeffort - TireLon0TFr * TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r);
  double TireLon1Ipe = Seeffort - TireLon1TFr * TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r);
  double TireLon2Ipe = Seeffort - TireLon2TFr * TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r);
  double TireLon3Ipe = Seeffort - TireLon3TFr * TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r);
  double Ixpe = ((((((TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c10r) + TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r)) + TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r)) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c17r + (-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c14r))) + GainK * (Jzstate / Jzi) * (Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c13r;
  double Iype = ((((-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c04r - TireLat2Rr * ((Jzstate / Jzi) * TF_c26r + Iystate / Iyi)) - TireLat1Rr * ((Jzstate / Jzi) * TF_c25r + Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c03r + TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c00r) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c07r) - GainK * (Jzstate / Jzi) * (Ixstate / Ixi);
  double Jzpe = ((((((((TireLat3Rr * (-((((Jzstate / Jzi) * TF_c27r) + ((Ixstate / Ixi) * TF_c17r)) + ((Iystate / Iyi) * TF_c07r)))) * TF_c27r) + ((TireLon0Rr * ((TireLon0TFr * (TireLon0Istate / TireLon0Ii)) - ((((Iystate / Iyi) * TF_c00r) + ((Ixstate / Ixi) * TF_c10r)) + ((Jzstate / Jzi) * TF_c20r)))) * TF_c20r)) + ((TireLon1Rr * ((TireLon1TFr * (TireLon1Istate / TireLon1Ii)) - ((Ixstate / Ixi) + ((Jzstate / Jzi) * TF_c21r)))) * TF_c21r)) + ((TireLon2Rr * ((TireLon2TFr * (TireLon2Istate / TireLon2Ii)) - ((Ixstate / Ixi) + ((Jzstate / Jzi) * TF_c22r)))) * TF_c22r)) + ((TireLon3Rr * ((TireLon3TFr * (TireLon3Istate / TireLon3Ii)) - ((((Iystate / Iyi) * TF_c03r) + ((Ixstate / Ixi) * TF_c13r)) + ((Jzstate / Jzi) * TF_c23r)))) * TF_c23r)) + ((TireLat0Rr * (-((((Jzstate / Jzi) * TF_c24r) + ((Ixstate / Ixi) * TF_c14r)) + ((Iystate / Iyi) * TF_c04r)))) * TF_c24r)) + ((TireLat1Rr * (-(((Jzstate / Jzi) * TF_c25r) + (Iystate / Iyi)))) * TF_c25r)) + ((TireLat2Rr * (-(((Jzstate / Jzi) * TF_c26r) + (Iystate / Iyi)))) * TF_c26r);

  double V = std::sqrt(std::pow(vx,2)+std::pow(vy,2));

  Eigen::VectorXd y_dot(y_t);

  y_dot[0] = Ixpe/Ixi;
  y_dot[1] = Iype/Iyi;
  y_dot[2] = Jzpe/Jzi;
  y_dot[3] = TireLon0Ipe;
  y_dot[4] = TireLon1Ipe;
  y_dot[5] = TireLon2Ipe;
  y_dot[6] = TireLon3Ipe;
  // x_dot in inertial frame
  y_dot[7] = V*std::cos(chi+beta);
  // y_dot in inertial frame
  y_dot[8] = V*std::sin(chi+beta);
  // kai_dot = omega rotational speed
  y_dot[9] = Jzstate/Jzi;

  return y_dot;
}

#endif //CARODE_H__
