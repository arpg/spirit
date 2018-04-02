#include <eigen3/Eigen/Eigen>

// where theta is equal to sigma
Eigen::ArrayXd CarODE2(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
  double sigma = SP_PI/4;
  double theta = SP_PI/4;
  double TF_c00r = std::cos(sigma);
  double TF_c03r = std::cos(sigma);
  double TF_c04r = std::sin(sigma);
  double TF_c07r = std::sin(sigma);
  double TF_c10r = -std::sin(sigma);
  double TF_c13r = -std::sin(sigma);
  double TF_c14r = std::cos(sigma);
  double TF_c17r = std::cos(sigma);
  double TF_c20r = -std::cos(sigma+theta);
  double TF_c21r = -std::cos(theta);
  double TF_c22r = std::cos(theta);
  double TF_c23r = std::cos(theta-sigma);
  double TF_c24r = -std::cos(theta-sigma);
  double TF_c25r = std::sin(theta);
  double TF_c26r = std::sin(theta);
  double TF_c27r = -std::sin(theta-sigma);
  double Ixstate = y_t[0];
  double Iystate = y_t[1];
  double Jzstate = y_t[2];
  double TireLon0Istate = y_t[3];
  double TireLon1Istate = y_t[4];
  double TireLon2Istate = y_t[5];
  double TireLon3Istate = y_t[6];
  double GainK = 1;
  double Ixi = 1;
  double Iyi = 1;
  double Jzi = 1;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;
  double Seeffort = 1;
  double Friction = 1;
  double Radius = 1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;
  double TireLat0Rr = Friction;
  double TireLat1Rr = Friction;
  double TireLat2Rr = Friction;
  double TireLat3Rr = Friction;
  double TireLon0Rr = Friction;
  double TireLon1Rr = Friction;
  double TireLon2Rr = Friction;
  double TireLon3Rr = Friction;

  //////////////////////

  //static equations:
  double Sepe = Seeffort;

  //dynamic equations:
  double Ixpf = Ixstate / Ixi;
  double Iypf = Iystate / Iyi;
  double Jzpf = Jzstate / Jzi;
  double TireLon0Ipf = TireLon0Istate / TireLon0Ii;
  double TireLon1Ipf = TireLon1Istate / TireLon1Ii;
  double TireLon2Ipf = TireLon2Istate / TireLon2Ii;
  double TireLon3Ipf = TireLon3Istate / TireLon3Ii;
  double Gainoutput = GainK * Jzpf;
  double TF_c00p1f = Iypf / TF_c00r;
  double TF_c03p1f = Iypf / TF_c03r;
  double TF_c04p1f = Iypf / TF_c04r;
  double TF_c10p1f = Ixpf / TF_c10r;
  double TF_c13p1f = Ixpf / TF_c13r;
  double TF_c14p1f = Ixpf / TF_c14r;
  double TF_c17p1f = Ixpf / TF_c17r;
  double TF_c21p1f = Jzpf / TF_c21r;
  double TF_c22p1f = Jzpf / TF_c22r;
  double TF_c23p1f = Jzpf / TF_c23r;
  double TF_c24p1f = Jzpf / TF_c24r;
  double TF_c25p1f = Jzpf / TF_c25r;
  double TF_c26p1f = Jzpf / TF_c26r;
  double TireLon0TFp2f = TireLon0TFr * TireLon0Ipf;
  double TireLon1TFp2f = TireLon1TFr * TireLon1Ipf;
  double TireLon2TFp2f = TireLon2TFr * TireLon2Ipf;
  double TireLon3TFp2f = TireLon3TFr * TireLon3Ipf;
  double Zero_enginep1v = ((TireLon3Ipf + TireLon2Ipf) + TireLon1Ipf) + TireLon0Ipf;
  double MGYp1e = Gainoutput * Iypf;
  double MGYp2e = Gainoutput * Ixpf;
  double TF_c07p1f = Iypf / TF_c07r;
  double Zero_lat0p1v = (TF_c24p1f + TF_c14p1f) + TF_c04p1f;
  double Zero_lat1p1v = TF_c25p1f + Ixpf;
  double Zero_lat2p1v = TF_c26p1f + Ixpf;
  double Zero_lon0p1v = TF_c10p1f + TF_c00p1f;
  double Zero_lon1p1v = TF_c21p1f + Iypf;
  double Zero_lon2p1v = TF_c22p1f + Iypf;
  double Zero_lon3p1v = (TF_c23p1f + TF_c13p1f) + TF_c03p1f;
  double Zero_lat3p1v = TF_c17p1f + TF_c07p1f;
  double TireLat0ZeroJunctionp1v = -Zero_lat0p1v;
  double TireLat1ZeroJunctionp1v = -Zero_lat1p1v;
  double TireLat2ZeroJunctionp1v = -Zero_lat2p1v;
  double TireLon0ZeroJunctionp2v = TireLon0TFp2f - Zero_lon0p1v;
  double TireLon1ZeroJunctionp2v = TireLon1TFp2f - Zero_lon1p1v;
  double TireLon2ZeroJunctionp2v = TireLon2TFp2f - Zero_lon2p1v;
  double TireLon3ZeroJunctionp2v = TireLon3TFp2f - Zero_lon3p1v;
  double TireLat0R1pe = TireLat0Rr * TireLat0ZeroJunctionp1v;
  double TireLat1R1pe = TireLat1Rr * TireLat1ZeroJunctionp1v;
  double TireLat2R1pe = TireLat2Rr * TireLat2ZeroJunctionp1v;
  double TireLat3ZeroJunctionp1v = -Zero_lat3p1v;
  double TireLon0Rpe = TireLon0Rr * TireLon0ZeroJunctionp2v;
  double TireLon1Rpe = TireLon1Rr * TireLon1ZeroJunctionp2v;
  double TireLon2Rpe = TireLon2Rr * TireLon2ZeroJunctionp2v;
  double TireLon3Rpe = TireLon3Rr * TireLon3ZeroJunctionp2v;
  double TireLat3R1pe = TireLat3Rr * TireLat3ZeroJunctionp1v;
  double TireLon0TFp1e = TireLon0TFr * TireLon0Rpe;
  double TireLon1TFp1e = TireLon1TFr * TireLon1Rpe;
  double TireLon2TFp1e = TireLon2TFr * TireLon2Rpe;
  double TireLon3TFp1e = TireLon3TFr * TireLon3Rpe;
  double TF_c00p2e = TireLon0Rpe / TF_c00r;
  double TF_c03p2e = TireLon3Rpe / TF_c03r;
  double TF_c04p2e = TireLat0R1pe / TF_c04r;
  double TF_c10p2e = TireLon0Rpe / TF_c10r;
  double TF_c13p2e = TireLon3Rpe / TF_c13r;
  double TF_c14p2e = TireLat0R1pe / TF_c14r;
  double TF_c21p2e = TireLon1Rpe / TF_c21r;
  double TF_c22p2e = TireLon2Rpe / TF_c22r;
  double TF_c23p2e = TireLon3Rpe / TF_c23r;
  double TF_c24p2e = TireLat0R1pe / TF_c24r;
  double TF_c25p2e = TireLat1R1pe / TF_c25r;
  double TF_c26p2e = TireLat2R1pe / TF_c26r;
  double TireLon0Ipe = Sepe - TireLon0TFp1e;
  double TireLon1Ipe = Sepe - TireLon1TFp1e;
  double TireLon2Ipe = Sepe - TireLon2TFp1e;
  double TireLon3Ipe = Sepe - TireLon3TFp1e;
  double Jzpe = ((((TF_c26p2e + TF_c25p2e) + TF_c24p2e) + TF_c23p2e) + TF_c22p2e) + TF_c21p2e;
  double TF_c07p2e = TireLat3R1pe / TF_c07r;
  double TF_c17p2e = TireLat3R1pe / TF_c17r;
  double Ixpe = (((((TireLat2R1pe + TireLat1R1pe) + TF_c17p2e) + TF_c14p2e) + TF_c13p2e) + TF_c10p2e) - MGYp1e;
  double Iype = (((((MGYp2e + TireLon2Rpe) + TireLon1Rpe) + TF_c04p2e) + TF_c03p2e) + TF_c00p2e) + TF_c07p2e;
  //////////////////////

  //system equations:
  Ixstate = Ixpe;
  Iystate = Iype;
  Jzstate = Jzpe;
  TireLon0Istate = TireLon0Ipe;
  TireLon1Istate = TireLon1Ipe;
  TireLon2Istate = TireLon2Ipe;
  TireLon3Istate = TireLon3Ipe;

  y_dot[0] = Ixstate;
  y_dot[1] = Iystate;
  y_dot[2] = Jzstate;
  y_dot[3] = TireLon0Istate;
  y_dot[4] = TireLon1Istate;
  y_dot[5] = TireLon2Istate;
  y_dot[6] = TireLon3Istate;
  return y_dot;
}

// where theta is not equal to sigma and sigma is non zero
Eigen::ArrayXd CarODE1(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
  double sigma = -SP_PI/6;
//  std::cout << "sigma " << sigma << std::endl;
  double theta = SP_PI/4.0;
  double epsilon = 1e-10;
  double TF_c00r = std::cos(sigma);
  double TF_c03r = std::cos(sigma);
  double TF_c04r = std::sin(sigma);
  double TF_c07r = std::sin(sigma);
  double TF_c10r = -std::sin(sigma);
  double TF_c13r = -std::sin(sigma);
  double TF_c14r = std::cos(sigma);
  double TF_c17r = std::cos(sigma);
  double TF_c20r = -std::cos(sigma+theta);
  double TF_c21r = -std::cos(theta);
  double TF_c22r = std::cos(theta);
  double TF_c23r = std::cos(theta-sigma);
  double TF_c24r = -std::cos(theta-sigma);
  double TF_c25r = std::sin(theta);
  double TF_c26r = std::sin(theta);
  double TF_c27r = -std::sin(theta-sigma);
  if(std::abs(TF_c00r)<epsilon) TF_c00r = 0;
  if(std::abs(TF_c03r)<epsilon) TF_c03r = 0;
  if(std::abs(TF_c04r)<epsilon) TF_c04r = 0;
  if(std::abs(TF_c07r)<epsilon) TF_c07r = 0;
  if(std::abs(TF_c10r)<epsilon) TF_c10r = 0;
  if(std::abs(TF_c13r)<epsilon) TF_c13r = 0;
  if(std::abs(TF_c14r)<epsilon) TF_c14r = 0;
  if(std::abs(TF_c17r)<epsilon) TF_c17r = 0;
  if(std::abs(TF_c20r)<epsilon) TF_c20r = 0;
  if(std::abs(TF_c21r)<epsilon) TF_c21r = 0;
  if(std::abs(TF_c22r)<epsilon) TF_c22r = 0;
  if(std::abs(TF_c23r)<epsilon) TF_c23r = 0;
  if(std::abs(TF_c24r)<epsilon) TF_c24r = 0;
  if(std::abs(TF_c25r)<epsilon) TF_c25r = 0;
  if(std::abs(TF_c26r)<epsilon) TF_c26r = 0;
  if(std::abs(TF_c27r)<epsilon) TF_c27r = 0;
//  std::cout << "00-> " << TF_c00r << std::endl;
//  std::cout << "03-> " << TF_c03r << std::endl;
//  std::cout << "04-> " << TF_c04r << std::endl;
//  std::cout << "07-> " << TF_c07r << std::endl;
//  std::cout << "10-> " << TF_c10r << std::endl;
//  std::cout << "13-> " << TF_c13r << std::endl;
//  std::cout << "14-> " << TF_c14r << std::endl;
//  std::cout << "17-> " << TF_c17r << std::endl;
//  std::cout << "20-> " << TF_c20r << std::endl;
//  std::cout << "21-> " << TF_c21r << std::endl;
//  std::cout << "22-> " << TF_c22r << std::endl;
//  std::cout << "23-> " << TF_c23r << std::endl;
//  std::cout << "24-> " << TF_c24r << std::endl;
//  std::cout << "25-> " << TF_c25r << std::endl;
//  std::cout << "26-> " << TF_c26r << std::endl;
//  std::cout << "27-> " << TF_c27r << std::endl;
//  while(1);
  double Ixstate = y_t[0];
  double Iystate = y_t[1];
  double Jzstate = y_t[2];
  double TireLon0Istate = y_t[3];
  double TireLon1Istate = y_t[4];
  double TireLon2Istate = y_t[5];
  double TireLon3Istate = y_t[6];
  double GainK = 12;
  double Ixi = 2;
  double Iyi = 2;
  double Jzi = 2;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;
  double Seeffort = -5;
  double Friction = 1;
  double Radius = 1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;
  double TireLat0Rr = Friction;
  double TireLat1Rr = Friction;
  double TireLat2Rr = Friction;
  double TireLat3Rr = Friction;
  double TireLon0Rr = Friction;
  double TireLon1Rr = Friction;
  double TireLon2Rr = Friction;
  double TireLon3Rr = Friction;

  //////////////////////
  double Sepe = Seeffort;

  double Ixpf = Ixstate / Ixi;
  double Iypf = Iystate / Iyi;
  double Jzpf = Jzstate / Jzi;
  double TireLon0Ipf = TireLon0Istate / TireLon0Ii;
  double TireLon1Ipf = TireLon1Istate / TireLon1Ii;
  double TireLon2Ipf = TireLon2Istate / TireLon2Ii;
  double TireLon3Ipf = TireLon3Istate / TireLon3Ii;
  double Gainoutput = GainK * Jzpf;
  double TF_c00p1f = 0;
  if(TF_c00r){
    TF_c00p1f = Iypf / TF_c00r;
  }
  double TF_c03p1f = 0;
  if(TF_c03r){
    TF_c03p1f = Iypf / TF_c03r;
  }
  double TF_c04p1f = 0;
  if(TF_c04r){
    TF_c04p1f = Iypf / TF_c04r;
  }
  double TF_c07p1f = 0;
  if(TF_c07r){
    TF_c07p1f = Iypf / TF_c07r;
  }
  double TF_c10p1f = 0;
  if(TF_c10r){
    TF_c10p1f = Ixpf / TF_c10r;
  }
  double TF_c13p1f = 0;
  if(TF_c13r){
    TF_c13p1f = Ixpf / TF_c13r;
  }
  double TF_c14p1f = 0;
  if(TF_c14r){
    TF_c14p1f = Ixpf / TF_c14r;
  }
  double TF_c17p1f = 0;
  if(TF_c17r){
    TF_c17p1f = Ixpf / TF_c17r;
  }
  double TF_c20p1f = 0;
  if(TF_c20r){
    TF_c20p1f = Jzpf / TF_c20r;
  }
  double TF_c21p1f = 0;
  if(TF_c21r){
    TF_c21p1f = Jzpf / TF_c21r;
  }
  double TF_c22p1f = 0;
  if(TF_c22r){
    TF_c22p1f = Jzpf / TF_c22r;
  }
  double TF_c23p1f = 0;
  if(TF_c23r){
    TF_c23p1f = Jzpf / TF_c23r;
  }
  double TF_c24p1f = 0;
  if(TF_c24r){
    TF_c24p1f = Jzpf / TF_c24r;
  }
  double TF_c25p1f = 0;
  if(TF_c25r){
    TF_c25p1f = Jzpf / TF_c25r;
  }
  double TF_c26p1f = 0;
  if(TF_c26r){
    TF_c26p1f = Jzpf / TF_c26r;
  }
  double TF_c27p1f = 0;
  if(TF_c27r){
    TF_c27p1f = Jzpf / TF_c27r;
  }
  double TireLon0TFp2f = TireLon0TFr * TireLon0Ipf;
  double TireLon1TFp2f = TireLon1TFr * TireLon1Ipf;
  double TireLon2TFp2f = TireLon2TFr * TireLon2Ipf;
  double TireLon3TFp2f = TireLon3TFr * TireLon3Ipf;
//  double ZeroJunction9p1v = ((TireLon3Ipf + TireLon2Ipf) + TireLon1Ipf) + TireLon0Ipf;
  double MGYp1e = Gainoutput * Ixpf;
  double MGYp2e = Gainoutput * Iypf;
  double ZeroJunction1p1v = (TF_c20p1f + TF_c10p1f) + TF_c00p1f;
  double ZeroJunction2p1v = TF_c21p1f + Iypf;
  double ZeroJunction3p1v = TF_c22p1f + Iypf;
  double ZeroJunction4p1v = (TF_c23p1f + TF_c13p1f) + TF_c03p1f;
  double ZeroJunction5p1v = (TF_c24p1f + TF_c14p1f) + TF_c04p1f;
  double ZeroJunction6p1v = TF_c25p1f + Ixpf;
  double ZeroJunction7p1v = TF_c26p1f + Ixpf;
  double ZeroJunction8p1v = (TF_c27p1f + TF_c17p1f) + TF_c07p1f;
  double TireLat0ZeroJunctionp2v = -ZeroJunction5p1v;
  double TireLat1ZeroJunctionp2v = -ZeroJunction6p1v;
  double TireLat2ZeroJunctionp2v = -ZeroJunction7p1v;
  double TireLat3ZeroJunctionp2v = -ZeroJunction8p1v;
  double TireLon0ZeroJunctionp1v = TireLon0TFp2f - ZeroJunction1p1v;
  double TireLon1ZeroJunctionp1v = TireLon1TFp2f - ZeroJunction2p1v;
  double TireLon2ZeroJunctionp1v = TireLon2TFp2f - ZeroJunction3p1v;
  double TireLon3ZeroJunctionp1v = TireLon3TFp2f - ZeroJunction4p1v;
  double TireLat0Rpe = TireLat0Rr * TireLat0ZeroJunctionp2v;
  double TireLat1Rpe = TireLat1Rr * TireLat1ZeroJunctionp2v;
  double TireLat2Rpe = TireLat2Rr * TireLat2ZeroJunctionp2v;
  double TireLat3Rpe = TireLat3Rr * TireLat3ZeroJunctionp2v;
  double TireLon0Rpe = TireLon0Rr * TireLon0ZeroJunctionp1v;
  double TireLon1Rpe = TireLon1Rr * TireLon1ZeroJunctionp1v;
  double TireLon2Rpe = TireLon2Rr * TireLon2ZeroJunctionp1v;
  double TireLon3Rpe = TireLon3Rr * TireLon3ZeroJunctionp1v;
  double TireLon0TFp1e = TireLon0TFr * TireLon0Rpe;
  double TireLon1TFp1e = TireLon1TFr * TireLon1Rpe;
  double TireLon2TFp1e = TireLon2TFr * TireLon2Rpe;
  double TireLon3TFp1e = TireLon3TFr * TireLon3Rpe;
  double TF_c00p2e = 0;
  if(TF_c00r){
    TF_c00p2e = TireLon0Rpe / TF_c00r;
  }
  double TF_c03p2e = 0;
  if(TF_c03r){
    TF_c03p2e = TireLon3Rpe / TF_c03r;
  }
  double TF_c04p2e = 0;
  if(TF_c04r){
    TF_c04p2e = TireLat0Rpe / TF_c04r;
  }
  double TF_c07p2e = 0;
  if(TF_c07r){
    TF_c07p2e = TireLat3Rpe / TF_c07r;
  }
  double TF_c10p2e = 0;
  if(TF_c10r){
    TF_c10p2e = TireLon0Rpe / TF_c10r;
  }
  double TF_c13p2e = 0;
  if(TF_c13r){
    TF_c13p2e = TireLon3Rpe / TF_c13r;
  }
  double TF_c14p2e = 0;
  if(TF_c14r){
    TF_c14p2e = TireLat0Rpe / TF_c14r;
  }
  double TF_c17p2e = 0;
  if(TF_c17r){
    TF_c17p2e = TireLat3Rpe / TF_c17r;
  }
  double TF_c20p2e = 0;
  if(TF_c20r){
    TF_c20p2e = TireLon0Rpe / TF_c20r;
  }
  double TF_c21p2e = 0;
  if(TF_c21r){
    TF_c21p2e = TireLon1Rpe / TF_c21r;
  }
  double TF_c22p2e = 0;
  if(TF_c22r){
    TF_c22p2e = TireLon2Rpe / TF_c22r;
  }
  double TF_c23p2e = 0;
  if(TF_c23r){
    TF_c23p2e = TireLon3Rpe / TF_c23r;
  }
  double TF_c24p2e = 0;
  if(TF_c24r){
    TF_c24p2e = TireLat0Rpe / TF_c24r;
  }
  double TF_c25p2e = 0;
  if(TF_c25r){
    TF_c25p2e = TireLat1Rpe / TF_c25r;
  }
  double TF_c26p2e = 0;
  if(TF_c26r){
    TF_c26p2e = TireLat2Rpe / TF_c26r;
  }
  double TF_c27p2e = 0;
  if(TF_c27r){
    TF_c27p2e = TireLat3Rpe / TF_c27r;
  }
  double TireLon0Ipe = Sepe - TireLon0TFp1e;
  double TireLon1Ipe = Sepe - TireLon1TFp1e;
  double TireLon2Ipe = Sepe - TireLon2TFp1e;
  double TireLon3Ipe = Sepe - TireLon3TFp1e;
  double Jzpe = ((((((TF_c27p2e + TF_c26p2e) + TF_c25p2e) + TF_c24p2e) + TF_c23p2e) + TF_c22p2e) + TF_c21p2e) + TF_c20p2e;
  double Ixpe = (((((MGYp2e + TireLat2Rpe) + TireLat1Rpe) + TF_c17p2e) + TF_c14p2e) + TF_c13p2e) + TF_c10p2e;
  double Iype = (((((TireLon2Rpe + TireLon1Rpe) + TF_c07p2e) + TF_c04p2e) + TF_c03p2e) + TF_c00p2e) - MGYp1e;

  //////////////////////
//  while(1);

  //system equations:
  Ixstate = Ixpe;
  Iystate = Iype;
  Jzstate = Jzpe;
  TireLon0Istate = TireLon0Ipe;
  TireLon1Istate = TireLon1Ipe;
  TireLon2Istate = TireLon2Ipe;
  TireLon3Istate = TireLon3Ipe;

  y_dot[0] = Ixstate;
  y_dot[1] = Iystate;
  y_dot[2] = Jzstate;
//  std::cout << "res -> " << Ixstate << " , " << Iystate << " , " << Jzstate  << std::endl;
  y_dot[3] = TireLon0Istate;
  y_dot[4] = TireLon1Istate;
  y_dot[5] = TireLon2Istate;
  y_dot[6] = TireLon3Istate;
  return y_dot;
}

// where sigma is equal to zero
Eigen::ArrayXd CarODE0(Eigen::VectorXd y_t) {
  Eigen::VectorXd y_dot(y_t);
  double sigma = SP_PI/4;
  double theta = SP_PI/4;
  double TF_c00r = std::cos(sigma);
  double TF_c03r = std::cos(sigma);
  double TF_c04r = std::sin(sigma);
  double TF_c07r = std::sin(sigma);
  double TF_c10r = -std::sin(sigma);
  double TF_c13r = -std::sin(sigma);
  double TF_c14r = std::cos(sigma);
  double TF_c17r = std::cos(sigma);
  double TF_c20r = -std::cos(sigma+theta);
  double TF_c21r = -std::cos(theta);
  double TF_c22r = std::cos(theta);
  double TF_c23r = std::cos(theta-sigma);
  double TF_c24r = -std::cos(theta-sigma);
  double TF_c25r = std::sin(theta);
  double TF_c26r = std::sin(theta);
  double TF_c27r = -std::sin(theta-sigma);
  double Ixstate = y_t[0];
  double Iystate = y_t[1];
  double Jzstate = y_t[2];
  double TireLon0Istate = y_t[3];
  double TireLon1Istate = y_t[4];
  double TireLon2Istate = y_t[5];
  double TireLon3Istate = y_t[6];
  double GainK = 1;
  double Ixi = 1;
  double Iyi = 1;
  double Jzi = 1;
  double TireLon0Ii = 1;
  double TireLon1Ii = 1;
  double TireLon2Ii = 1;
  double TireLon3Ii = 1;
  double Seeffort = 1;
  double Friction = 1;
  double Radius = 1;
  double TireLon0TFr = Radius;
  double TireLon1TFr = Radius;
  double TireLon2TFr = Radius;
  double TireLon3TFr = Radius;
  double TireLat0Rr = Friction;
  double TireLat1Rr = Friction;
  double TireLat2Rr = Friction;
  double TireLat3Rr = Friction;
  double TireLon0Rr = Friction;
  double TireLon1Rr = Friction;
  double TireLon2Rr = Friction;
  double TireLon3Rr = Friction;

  //////////////////////
  double Sepe = Seeffort;
  double Ixpf = Ixstate / Ixi;
  double Iypf = Iystate / Iyi;
  double Jzpf = Jzstate / Jzi;
  double TireLon0Ipf = TireLon0Istate / TireLon0Ii;
  double TireLon1Ipf = TireLon1Istate / TireLon1Ii;
  double TireLon2Ipf = TireLon2Istate / TireLon2Ii;
  double TireLon3Ipf = TireLon3Istate / TireLon3Ii;
  double Gainoutput = GainK * Jzpf;
  double TF_c00p1f = Iypf / TF_c00r;
  double TF_c03p1f = Iypf / TF_c03r;
  double TF_c14p1f = Ixpf / TF_c14r;
  double TF_c17p1f = Ixpf / TF_c17r;
  double TF_c20p1f = Jzpf / TF_c20r;
  double TF_c21p1f = Jzpf / TF_c21r;
  double TF_c22p1f = Jzpf / TF_c22r;
  double TF_c23p1f = Jzpf / TF_c23r;
  double TF_c24p1f = Jzpf / TF_c24r;
  double TF_c25p1f = Jzpf / TF_c25r;
  double TF_c26p1f = Jzpf / TF_c26r;
  double TF_c27p1f = Jzpf / TF_c27r;
  double TireLon0TFp2f = TireLon0TFr * TireLon0Ipf;
  double TireLon1TFp2f = TireLon1TFr * TireLon1Ipf;
  double TireLon2TFp2f = TireLon2TFr * TireLon2Ipf;
  double TireLon3TFp2f = TireLon3TFr * TireLon3Ipf;
  double ZeroJunction9p1v = ((TireLon3Ipf + TireLon2Ipf) + TireLon1Ipf) + TireLon0Ipf;
  double MGYp1e = Gainoutput * Ixpf;
  double MGYp2e = Gainoutput * Iypf;
  double ZeroJunction1p1v = TF_c20p1f + TF_c00p1f;
  double ZeroJunction2p1v = TF_c21p1f + Iypf;
  double ZeroJunction3p1v = TF_c22p1f + Iypf;
  double ZeroJunction4p1v = TF_c23p1f + TF_c03p1f;
  double ZeroJunction5p1v = TF_c24p1f + TF_c14p1f;
  double ZeroJunction6p1v = TF_c25p1f + Ixpf;
  double ZeroJunction7p1v = TF_c26p1f + Ixpf;
  double ZeroJunction8p1v = TF_c27p1f + TF_c17p1f;
  double TireLat0ZeroJunctionp2v = -ZeroJunction5p1v;
  double TireLat1ZeroJunctionp2v = -ZeroJunction6p1v;
  double TireLat2ZeroJunctionp2v = -ZeroJunction7p1v;
  double TireLat3ZeroJunctionp2v = -ZeroJunction8p1v;
  double TireLon0ZeroJunctionp1v = TireLon0TFp2f - ZeroJunction1p1v;
  double TireLon1ZeroJunctionp1v = TireLon1TFp2f - ZeroJunction2p1v;
  double TireLon2ZeroJunctionp1v = TireLon2TFp2f - ZeroJunction3p1v;
  double TireLon3ZeroJunctionp1v = TireLon3TFp2f - ZeroJunction4p1v;
  double TireLat0Rpe = TireLat0Rr * TireLat0ZeroJunctionp2v;
  double TireLat1Rpe = TireLat1Rr * TireLat1ZeroJunctionp2v;
  double TireLat2Rpe = TireLat2Rr * TireLat2ZeroJunctionp2v;
  double TireLat3Rpe = TireLat3Rr * TireLat3ZeroJunctionp2v;
  double TireLon0Rpe = TireLon0Rr * TireLon0ZeroJunctionp1v;
  double TireLon1Rpe = TireLon1Rr * TireLon1ZeroJunctionp1v;
  double TireLon2Rpe = TireLon2Rr * TireLon2ZeroJunctionp1v;
  double TireLon3Rpe = TireLon3Rr * TireLon3ZeroJunctionp1v;
  double TireLon0TFp1e = TireLon0TFr * TireLon0Rpe;
  double TireLon1TFp1e = TireLon1TFr * TireLon1Rpe;
  double TireLon2TFp1e = TireLon2TFr * TireLon2Rpe;
  double TireLon3TFp1e = TireLon3TFr * TireLon3Rpe;
  double TF_c00p2e = TireLon0Rpe / TF_c00r;
  double TF_c03p2e = TireLon3Rpe / TF_c03r;
  double TF_c14p2e = TireLat0Rpe / TF_c14r;
  double TF_c17p2e = TireLat3Rpe / TF_c17r;
  double TF_c20p2e = TireLon0Rpe / TF_c20r;
  double TF_c21p2e = TireLon1Rpe / TF_c21r;
  double TF_c22p2e = TireLon2Rpe / TF_c22r;
  double TF_c23p2e = TireLon3Rpe / TF_c23r;
  double TF_c24p2e = TireLat0Rpe / TF_c24r;
  double TF_c25p2e = TireLat1Rpe / TF_c25r;
  double TF_c26p2e = TireLat2Rpe / TF_c26r;
  double TF_c27p2e = TireLat3Rpe / TF_c27r;
  double TireLon0Ipe = Sepe - TireLon0TFp1e;
  double TireLon1Ipe = Sepe - TireLon1TFp1e;
  double TireLon2Ipe = Sepe - TireLon2TFp1e;
  double TireLon3Ipe = Sepe - TireLon3TFp1e;
  double Jzpe = ((((((TF_c27p2e + TF_c26p2e) + TF_c25p2e) + TF_c24p2e) + TF_c23p2e) + TF_c22p2e) + TF_c21p2e) + TF_c20p2e;
  double Ixpe = (((MGYp2e + TireLat2Rpe) + TireLat1Rpe) + TF_c17p2e) + TF_c14p2e;
  double Iype = (((TireLon2Rpe + TireLon1Rpe) + TF_c03p2e) + TF_c00p2e) - MGYp1e;


  //////////////////////

  //system equations:
  Ixstate = Ixpe;
  Iystate = Iype;
  Jzstate = Jzpe;
  TireLon0Istate = TireLon0Ipe;
  TireLon1Istate = TireLon1Ipe;
  TireLon2Istate = TireLon2Ipe;
  TireLon3Istate = TireLon3Ipe;

  y_dot[0] = Ixstate;
  y_dot[1] = Iystate;
  y_dot[2] = Jzstate;
  y_dot[3] = TireLon0Istate;
  y_dot[4] = TireLon1Istate;
  y_dot[5] = TireLon2Istate;
  y_dot[6] = TireLon3Istate;
  return y_dot;
}

