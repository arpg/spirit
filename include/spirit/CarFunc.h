#include <cmath>

void CarFunc(double x_t0, double x_t1, double x_t2, double x_t3, double x_t4, double x_t5, double x_t6, double& x_dot0, double& x_dot1, double& x_dot2, double& x_dot3, double& x_dot4, double& x_dot5, double& x_dot6, double sigma, double tau ) {
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
  double Ixstate = x_t0;
  double Iystate = x_t1;
  double Jzstate = x_t2;
  double TireLon0Istate = x_t3;
  double TireLon1Istate = x_t4;
  double TireLon2Istate = x_t5;
  double TireLon3Istate = x_t6;
  double Ixi = 2;
  double Iyi = 2;
  double Jzi = 2;
  double vx = x_t0/Ixi;
  double GainK = 12;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;
  double Seeffort = tau;
  double Radius = 0.1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;

  // longitudinal friction
  double xmagic_d = 100.68;
  double xmagic_b = 8.22;
  double xmagic_c = 1.65;
  double xmagic_e = -10;

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

  x_dot0 = Ixpe;
  x_dot1 = Iype;
  x_dot2 = Jzpe;
  x_dot3 = TireLon0Ipe;
  x_dot4 = TireLon1Ipe;
  x_dot5 = TireLon2Ipe;
  x_dot6 = TireLon3Ipe;

}
