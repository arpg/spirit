#include <eigen3/Eigen/Eigen>

Eigen::ArrayXd CarODE(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
  double sigma = SP_PI/10;
  double theta = SP_PI/4.0;
  double TF_c00r = std::sin(sigma);
  double TF_c03r = std::sin(sigma);
  double TF_c04r = std::cos(sigma);
  double TF_c07r = std::cos(sigma);
  double TF_c10r = std::cos(sigma);
  double TF_c13r = std::cos(sigma);
  double TF_c14r = -std::sin(sigma);
  double TF_c17r = -std::sin(sigma);
  double TF_c20r = std::cos(-sigma+theta);
  double TF_c21r = std::cos(theta);
  double TF_c22r = -std::cos(theta);
  double TF_c23r = -std::cos(theta+sigma);
  double TF_c24r = std::sin(theta-sigma);
  double TF_c25r = -std::sin(theta);
  double TF_c26r = -std::sin(theta);
  double TF_c27r = std::sin(sigma+theta);

  double Ixstate = y_t[0];
  double Iystate = y_t[1];
  double Jzstate = y_t[2];
  double TireLon0Istate = y_t[3];
  double TireLon1Istate = y_t[4];
  double TireLon2Istate = y_t[5];
  double TireLon3Istate = y_t[6];
  double Ixi = 2;
  double Iyi = 2;
  double Jzi = 2;
  double vx = y_t[0]/Ixi;
  double vy = y_t[1]/Iyi;
  double chi = y_t[9]/Jzi;
  double V = std::sqrt(std::pow(vx,2)+std::pow(vy,2));
  double beta = std::atan2(vy,vx);

  double GainK = 12;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;
  double Seeffort = 1;
  double LonFriction = 100;
  double LatFriction = 100;
  double Radius = 0.1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;

  double xmagic_d = 100.68;
  double xmagic_b = 8.22;
  double xmagic_c = 1.65;
  double xmagic_e = -10;

  // longitudinal friction
  double rw0 = Radius * TireLon0Istate;
  double t0_vlon = vx * std::cos(sigma);
  double k0 = std::abs( rw0 - t0_vlon)/std::max(std::abs(rw0),std::abs(t0_vlon));
  double TireLon0Rr = xmagic_d*std::sin(xmagic_c*std::atan(xmagic_b*(1-xmagic_e)*k0+xmagic_e*std::atan(xmagic_b*k0)));

  double rw1 = Radius * TireLon1Istate;
  double t1_vlon = vx;
  double k1 = std::abs( rw1 - t1_vlon)/std::max(std::abs(rw1),std::abs(t1_vlon));
  double TireLon1Rr = xmagic_d*std::sin(xmagic_c*std::atan(xmagic_b*(1-xmagic_e)*k1+xmagic_e*std::atan(xmagic_b*k1)));

  double rw2 = Radius * TireLon2Istate;
  double t2_vlon = vx;
  double k2 = std::abs( rw2 - t2_vlon)/std::max(std::abs(rw2),std::abs(t2_vlon));
  double TireLon2Rr = xmagic_d*std::sin(xmagic_c*std::atan(xmagic_b*(1-xmagic_e)*k2+xmagic_e*std::atan(xmagic_b*k2)));

  double rw3 = Radius * TireLon3Istate;
  double t3_vlon = vx * std::cos(sigma);
  double k3 = std::abs( rw3 - t3_vlon)/std::max(std::abs(rw3),std::abs(t3_vlon));
  double TireLon3Rr = xmagic_d*std::sin(xmagic_c*std::atan(xmagic_b*(1-xmagic_e)*k3+xmagic_e*std::atan(xmagic_b*k3)));

  if(std::isnan(k0)||std::isnan(k1)||std::isnan(k2)||std::isnan(k3)) {
    std::cerr << "Error in file:" << __FILE__ << " Line:" << __LINE__ << " " << "Both wheel and linear velocities cant be zero at the same time." << std::endl;
    std::exit(EXIT_FAILURE);
  }
  // lateral friction
  double ymagic_d = 100.68;
  double ymagic_b = 8.82;
  double ymagic_c = 1.79;
  double ymagic_e = -2.02;

  double bet0 = std::abs(theta - sigma);
  double TireLat0Rr = ymagic_d*std::sin(ymagic_c*std::atan(ymagic_b*(1-ymagic_e)*bet0+ymagic_e*std::atan(ymagic_b*bet0)));

  double bet1 = std::abs(sigma);
  double TireLat1Rr = ymagic_d*std::sin(ymagic_c*std::atan(ymagic_b*(1-ymagic_e)*bet1+ymagic_e*std::atan(ymagic_b*bet1)));

  double bet2 = std::abs(sigma);
  double TireLat2Rr = ymagic_d*std::sin(ymagic_c*std::atan(ymagic_b*(1-ymagic_e)*bet2+ymagic_e*std::atan(ymagic_b*bet2)));

  double bet3 = std::abs(theta - sigma);
  double TireLat3Rr = ymagic_d*std::sin(ymagic_c*std::atan(ymagic_b*(1-ymagic_e)*bet3+ymagic_e*std::atan(ymagic_b*bet3)));

#if 1
  double Sepe = Seeffort;
  double TireLon0Ipf = TireLon0Istate / TireLon0Ii;
  double TireLon1Ipf = TireLon1Istate / TireLon1Ii;
  double TireLon2Ipf = TireLon2Istate / TireLon2Ii;
  double TireLon3Ipf = TireLon3Istate / TireLon3Ii;
  double Jzpf = Jzstate / Jzi;
  double Ixpf = Ixstate / Ixi;
  double Iypf = Iystate / Iyi;
  double TireLon1TFp2f = TireLon1TFr * TireLon1Ipf;
  double TireLon2TFp2f = TireLon2TFr * TireLon2Ipf;
  double TireLon3TFp2f = TireLon3TFr * TireLon3Ipf;
  double ZeroJunctionp5v = ((TireLon0Ipf + TireLon3Ipf) + TireLon2Ipf) + TireLon1Ipf;
  if(ZeroJunctionp5v >10) {
    Sepe = 0;
  }
  double TF_c27p1f = Jzpf * TF_c27r;
  double TF_c04p1f = Iypf * TF_c04r;
  double TF_c10p1f = Ixpf * TF_c10r;
  double TF_c00p1f = Iypf * TF_c00r;
  double TF_c03p1f = Iypf * TF_c03r;
  double TF_c17p1f = Ixpf * TF_c17r;
  double TF_c14p1f = Ixpf * TF_c14r;
  double Gainoutput = GainK * Jzpf;
  double TireLon0TFp2f = TireLon0TFr * TireLon0Ipf;
  double MGYp1e = Gainoutput * Ixpf;
  double MGYp2e = Gainoutput * Iypf;
  double TF_c20p1f = Jzpf * TF_c20r;
  double TF_c21p1f = Jzpf * TF_c21r;
  double TF_c22p1f = Jzpf * TF_c22r;
  double TF_c23p1f = Jzpf * TF_c23r;
  double TF_c24p1f = Jzpf * TF_c24r;
  double TF_c25p1f = Jzpf * TF_c25r;
  double TF_c26p1f = Jzpf * TF_c26r;
  double TF_c07p1f = Iypf * TF_c07r;
  double TF_c13p1f = Ixpf * TF_c13r;
  double ZeroJunction2p1v = Ixpf + TF_c22p1f;
  double ZeroJunction3p1v = Ixpf + TF_c21p1f;
  double ZeroJunction4p1v = (TF_c00p1f + TF_c10p1f) + TF_c20p1f;
  double ZeroJunction5p2v = TF_c25p1f + Iypf;
  double ZeroJunction6p2v = (TF_c24p1f + TF_c14p1f) + TF_c04p1f;
  double ZeroJunction7p1v = (TF_c03p1f + TF_c13p1f) + TF_c23p1f;
  double ZeroJunction8p2v = (TF_c27p1f + TF_c17p1f) + TF_c07p1f;
  double ZeroJunction9p2v = TF_c26p1f + Iypf;
  double TireLon0ZeroJunction1p3v = TireLon0TFp2f - ZeroJunction4p1v;
  double TireLon1ZeroJunction1p3v = TireLon1TFp2f - ZeroJunction3p1v;
  double TireLon2ZeroJunction1p3v = TireLon2TFp2f - ZeroJunction2p1v;
  double TireLon3ZeroJunction1p3v = TireLon3TFp2f - ZeroJunction7p1v;
  double TireLat0ZeroJunctionp1v = -ZeroJunction6p2v;
  double TireLat1ZeroJunctionp2v = -ZeroJunction5p2v;
  double TireLat2ZeroJunctionp2v = -ZeroJunction9p2v;
  double TireLat3ZeroJunctionp2v = -ZeroJunction8p2v;
  double TireLon0Rpe = TireLon0Rr * TireLon0ZeroJunction1p3v;
  double TireLon1Rpe = TireLon1Rr * TireLon1ZeroJunction1p3v;
  double TireLon2Rpe = TireLon2Rr * TireLon2ZeroJunction1p3v;
  double TireLon3Rpe = TireLon3Rr * TireLon3ZeroJunction1p3v;
  double TireLat0Rpe = TireLat0Rr * TireLat0ZeroJunctionp1v;
  double TireLat1Rpe = TireLat1Rr * TireLat1ZeroJunctionp2v;
  double TireLat2Rpe = TireLat2Rr * TireLat2ZeroJunctionp2v;
  double TireLat3Rpe = TireLat3Rr * TireLat3ZeroJunctionp2v;
  double TireLon0TFp1e = TireLon0TFr * TireLon0Rpe;
  double TireLon1TFp1e = TireLon1TFr * TireLon1Rpe;
  double TireLon2TFp1e = TireLon2TFr * TireLon2Rpe;
  double TireLon3TFp1e = TireLon3TFr * TireLon3Rpe;
  double TireLon0Ipe = Sepe - TireLon0TFp1e;
  double TireLon1Ipe = Sepe - TireLon1TFp1e;
  double TireLon2Ipe = Sepe - TireLon2TFp1e;
  double TireLon3Ipe = Sepe - TireLon3TFp1e;
  double TF_c24p2e = TireLat0Rpe * TF_c24r;
  double TF_c25p2e = TireLat1Rpe * TF_c25r;
  double TF_c26p2e = TireLat2Rpe * TF_c26r;
  double TF_c27p2e = TireLat3Rpe * TF_c27r;
  double TF_c07p2e = TireLat3Rpe * TF_c07r;
  double TF_c04p2e = TireLat0Rpe * TF_c04r;
  double TF_c17p2e = TireLat3Rpe * TF_c17r;
  double TF_c14p2e = TireLat0Rpe * TF_c14r;
  double TF_c20p2e = TireLon0Rpe * TF_c20r;
  double TF_c21p2e = TireLon1Rpe * TF_c21r;
  double TF_c22p2e = TireLon2Rpe * TF_c22r;
  double TF_c23p2e = TireLon3Rpe * TF_c23r;
  double TF_c10p2e = TireLon0Rpe * TF_c10r;
  double TF_c13p2e = TireLon3Rpe * TF_c13r;
  double TF_c00p2e = TireLon0Rpe * TF_c00r;
  double TF_c03p2e = TireLon3Rpe * TF_c03r;
  double Iype = (((((TF_c04p2e + TireLat2Rpe) + TireLat1Rpe) + TF_c03p2e) + TF_c00p2e) + TF_c07p2e) - MGYp1e;
  double Ixpe = (((((TF_c10p2e + TireLon2Rpe) + TireLon1Rpe) + TF_c17p2e) + TF_c14p2e) + MGYp2e) + TF_c13p2e;
  double Jzpe = ((((((TF_c27p2e + TF_c20p2e) + TF_c21p2e) + TF_c22p2e) + TF_c23p2e) + TF_c24p2e) + TF_c25p2e) + TF_c26p2e;
#else
  double TireLon0Ipe = Seeffort - TireLon0TFr * TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r);
  double TireLon1Ipe = Seeffort - TireLon1TFr * TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r);
  double TireLon2Ipe = Seeffort - TireLon2TFr * TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r);
  double TireLon3Ipe = Seeffort - TireLon3TFr * TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r);
  double Jzpe = ((((((((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c27r + (TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c20r)) + (TireLon1Rr * TF_c21r * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r))) + (TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r) * TF_c22r)) + (TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c23r)) - TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c24r) - TireLat1Rr * ((Jzstate / Jzi) * TF_c25r + Iystate / Iyi) * TF_c25r) - TireLat2Rr * ((Jzstate / Jzi) * TF_c26r + Iystate / Iyi) * TF_c26r;
  double Ixpe = ((((((TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c10r) + TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r)) + TireLon1Rr * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r)) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c17r + (-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c14r))) + GainK * (Jzstate / Jzi) * (Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c13r;
  double Iype = ((((-TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c04r - TireLat2Rr * ((Jzstate / Jzi) * TF_c26r + Iystate / Iyi)) - TireLat1Rr * ((Jzstate / Jzi) * TF_c25r + Iystate / Iyi)) + TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c03r + TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c00r) + ((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c07r) - GainK * (Jzstate / Jzi) * (Ixstate / Ixi);

  // Simplified Notation
//  double w0_d = tau - r * mu_n0 * (r * (w0 / I_t0) - (p_y / I_y) * c00 - (p_x / I_x) * c10 - (p_omega / I_omega) * c20);
//  double w1_d = tau - r * mu_n1 * (r * (w1 / I_t1) - (p_x / I_x) - (p_omega / I_omega) * c21);
//  double w2_d = tau - r * mu_n2 * (r * (w2 / I_t2) - (p_x / I_x) - (p_omega / I_omega) * c22);
//  double w3_d = tau - r * mu_n3 * (r * (w3 / I_t3) - (p_y / I_y) * c03 - (p_x / I_x) * c13 - (p_omega / I_omega) * c23);
//  double p_omega_d = ((((((((p_omega / I_omega) * c27 + (p_x / I_x) * c17 + (p_y / I_y) * c07) * -mu_t3 * c27 + (mu_n0 * (r * (w0 / I_t0) - (p_y / I_y) * c00 - (p_x / I_x) * c10 - (p_omega / I_omega) * c20) * c20)) + (mu_n1 * c21 * (r * (w1 / I_t1) - (p_x / I_x) - (p_omega / I_omega) * c21))) + (mu_n2 * (r * (w2 / I_t2) - (p_x / I_x) - (p_omega / I_omega) * c22) * c22)) + (mu_n3 * (r * (w3 / I_t3) - (p_y / I_y) * c03 - (p_x / I_x) * c13 - (p_omega / I_omega) * c23) * c23)) - mu_t0 * ((p_omega / I_omega) * c24 + (p_x / I_x) * c14 + (p_y / I_y) * c04) * c24) - mu_t1 * ((p_omega / I_omega) * c25 + p_y / I_y) * c25) - mu_t2 * ((p_omega / I_omega) * c26 + p_y / I_y) * c26;
//  double p_x_d = ((((((mu_n0 * (r * (w0 / I_t0) - (p_y / I_y) * c00 - (p_x / I_x) * c10 - (p_omega / I_omega) * c20) * c10) + mu_n2 * (r * (w2 / I_t2) - (p_x / I_x) - (p_omega / I_omega) * c22)) + mu_n1 * (r * (w1 / I_t1) - (p_x / I_x) - (p_omega / I_omega) * c21)) + ((p_omega / I_omega) * c27 + (p_x / I_x) * c17 + (p_y / I_y) * c07) * -mu_t3 * c17 + (-mu_t0 * ((p_omega / I_omega) * c24 + (p_x / I_x) * c14 + (p_y / I_y) * c04) * c14))) + m * (p_omega / I_omega) * (p_y / I_y)) + mu_n3 * (r * (w3 / I_t3) - (p_y / I_y) * c03 - (p_x / I_x) * c13 - (p_omega / I_omega) * c23) * c13;
//  double p_y_d = ((((-mu_t0 * ((p_omega / I_omega) * c24 + (p_x / I_x) * c14 + (p_y / I_y) * c04) * c04 - mu_t2 * ((p_omega / I_omega) * c26 + p_y / I_y)) - mu_t1 * ((p_omega / I_omega) * c25 + p_y / I_y)) + mu_n3 * (r * (w3 / I_t3) - (p_y / I_y) * c03 - (p_x / I_x) * c13 - (p_omega / I_omega) * c23) * c03 + mu_n0 * (r * (w0 / I_t0) - (p_y / I_y) * c00 - (p_x / I_x) * c10 - (p_omega / I_omega) * c20) * c00) + ((p_omega / I_omega) * c27 + (p_x / I_x) * c17 + (p_y / I_y) * c07) * -mu_t3 * c07) - m * (p_omega / I_omega) * (p_x / I_x);

  // Latex form
//  \dot{w_0} = \tau - r * \mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20});
//  \dot{w_1} = \tau - r * \mu_{n1} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21});
//  \dot{w_2} = \tau - r * \mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22});
//  \dot{w_3} = \tau - r * \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23});
//  \dot{p_\omega} = ((((((((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{27} + (\mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{20})) + (\mu_{n1} * c_{21} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21}))) + (\mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22}) * c_{22})) + (\mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{23})) - \mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{24}) - \mu_{t1} * ((p_\omega / I_\omega) * c_{25} + p_y / I_y) * c_{25}) - \mu_{t2} * ((p_\omega / I_\omega) * c_{26} + p_y / I_y) * c_{26};
//  \dot{p_x} = ((((((\mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{10}) + \mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22})) + \mu_{n1} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21})) + ((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{17} + (-\mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{14}))) + m * (p_\omega / I_\omega) * (p_y / I_y)) + \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{13};
//  \dot{p_y} = ((((-\mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{04} - \mu_{t2} * ((p_\omega / I_\omega) * c_{26} + p_y / I_y)) - \mu_{t1} * ((p_\omega / I_\omega) * c_{25} + p_y / I_y)) + \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{03} + \mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{00}) + ((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{07}) - m * (p_\omega / I_\omega) * (p_x / I_x);

#endif

  y_dot[0] = Ixpe;
  y_dot[1] = Iype;
  y_dot[2] = Jzpe;
  y_dot[3] = TireLon0Ipe;
  y_dot[4] = TireLon1Ipe;
  y_dot[5] = TireLon2Ipe;
  y_dot[6] = TireLon3Ipe;
  y_dot[7] = V*std::sin(chi+beta);
  y_dot[8] = V*std::cos(chi+beta);
  y_dot[9] = Jzstate/Jzi;


  return y_dot;
}
