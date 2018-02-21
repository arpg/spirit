#ifndef CLF_H__
#define CLF_H__

double V(double th, double x, double y, double v, double p){
  return 0.372503132228*p + 1.97248006041*th*th + 2.75481825315*x*x + 2.59443611808*y*y + 2.46570845211*v*v - 2.21861909747*th*x - 0.252299355612*th*y + 0.279942258467*th*v + 0.0578921429474*x*y + 0.132761213741*v*x - 0.536165847515*v*y;
}

double V_pure(double th, double x, double y, double v){
  return 1.97248006041*th*th + 2.75481825315*x*x + 2.59443611808*y*y + 2.46570845211*v*v - 2.21861909747*th*x - 0.252299355612*th*y + 0.279942258467*th*v + 0.0578921429474*x*y + 0.132761213741*v*x - 0.536165847515*v*y;
}

double dV(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double v_r = 2;
  double dth_r = 0;
  double v_t = v + v_r;
  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);
  double r = (2*1.97248006041*th -2.21861909747*x - 0.252299355612 *y + 0.279942258467*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) + 
     (2*2.75481825315*x - 2.21861909747*th +0.0578921429474*y + 0.132761213741 *v)*(dth_r*(1+u_3)*y + v_t*-sin_th) + 
     (2*2.59443611808*y - 0.252299355612 *th +0.0578921429474*x - 0.536165847515*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) + 
     (2*2.46570845211*v + 0.279942258467*th + 0.132761213741 *x - 0.536165847515*y)*(u_1);
  double boundary = 1;
  double L = V(th, x, y, v, p);
  if( L > boundary )
    // r += 0.249999989812*(1+u_3);
    r += 0.372503132228*(1+u_3);
  return r;
}

double rem(double a, double b){
  double q = a / b;
  return a - ((int)q)*b;
}

double rem(double a, int b){
  double dec = a - (int) a;
  return ((int)a % b) + dec;
}

class Input{
  public: double u_1;
  public: double u_2;
  public: double u_3;
  public: int seg;
  public: Input(double u_1, double u_2, double u_3, int seg){
    this->u_1 = u_1;
    this->u_2 = u_2;
    this->u_3 = u_3;
    this->seg = seg;
  }
};

Input K(double th_t, double x_t, double y_t, double v_t, double p_t, int seg_prev, double u_1_prev, double u_2_prev, double u_3_prev){
  
  // double horizon = 1.5;
  // int seg = (int)(rem(p_t, horizon*4)/horizon);
  // if(seg != seg_prev)
  //   std::cout << "\n\n\nsegment " << seg << "\n\n\n" << std::endl;
  int seg = seg_prev;

  double p = p_t;//rem(p_t, horizon);

  double new_x = x_t;
  double new_y = y_t;
  double new_th = th_t;
  // if (seg == 1){
  //     new_x = y_t;
  //     new_y = -x_t;
  //     new_th = th_t-(SP_PI/2);
  // }
  // if (seg == 2){
  //     new_x = -x_t;
  //     new_y = -y_t;
  //     new_th = th_t+SP_PI;
  //     // std::cout << new_th << ":" << rem((new_th+SP_PI), (2*SP_PI))-SP_PI << std::endl;
  // }
  // if (seg == 3){
  //     new_x = -y_t;
  //     new_y = x_t;
  //     new_th = th_t+(SP_PI/2);
  // }
  new_th = rem((new_th+SP_PI), (2*SP_PI))-SP_PI;

  x_t = new_x;
  y_t = new_y;
  th_t = new_th;

  // std::cout << "state: " << th_t << ", " << x_t << ", " << y_t << ", " << v_t << ", " << p << std::endl;

  double dth_r = 0;
  double th_r = 0 - SP_PI/2;
  double v_r = 2;
  double x_r = -4 + 2*p;
  double y_r = 0;
  double th_d = th_t - th_r;
  double v_d  = v_t  - v_r;
  double x_d  = x_t  - x_r;
  double y_d  = y_t  - y_r;
  double th = th_d;
  double v = v_d;
  double x = cos(-th_r)*x_d-sin(-th_r)*y_d;
  double y = sin(-th_r)*x_d+cos(-th_r)*y_d;

  double L = V(th, x, y, v, p);
  double Lpure = V_pure(th, x, y, v);
  std::cout << "L: " << Lpure << std::endl;

  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);

  double boundary = 1;

  double  b_1 = (2*2.46570845211*v + 0.279942258467*th + 0.132761213741 *x - 0.536165847515*y);
  double  b_2 = (2*1.97248006041*th -2.21861909747*x - 0.252299355612 *y + 0.279942258467*v)*(-2.9380*(v_t));
  double  b_3 = (2*1.97248006041*th -2.21861909747*x - 0.252299355612 *y + 0.279942258467*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
     (2*2.75481825315*x - 2.21861909747*th +0.0578921429474*y + 0.132761213741 *v)*(dth_r*(1+0)*y) + 
     (2*2.59443611808*y - 0.252299355612 *th +0.0578921429474*x - 0.536165847515*v)*(dth_r*(1+0)*(-x) - v_r*(1+0)) + 
     (2*2.46570845211*v + 0.279942258467*th + 0.132761213741 *x - 0.536165847515*y)*(0);
  if (L > boundary){
    // b_3 += 0.249999989812;
    b_3 += 0.372503132228;
  }
  double a = (2*1.97248006041*th -2.21861909747*x - 0.252299355612 *y + 0.279942258467*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
     (2*2.75481825315*x - 2.21861909747*th +0.0578921429474*y + 0.132761213741 *v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
     (2*2.59443611808*y - 0.252299355612 *th +0.0578921429474*x - 0.536165847515*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
     (2*2.46570845211*v + 0.279942258467*th + 0.132761213741 *x - 0.536165847515*y)*(0);
  if (L > boundary){
    // a += 0.249999989812;
    a += 0.372503132228;
  }
  double beta = b_1*b_1 + b_2*b_2 + b_3*b_3;
  
  double u1_max = 4;
  double u1_min = -4;
  double u2_max = 1;
  double u2_min = -1;
  double u3_max = 9;
  double u3_min = -0.9;

  double beta_low = 0.1/4;
  double u_1 = 0;
  double u_2 = 0;
  double u_3 = 0;

  double dL = 0;

std::cout << "beta:  " << beta <<  std::endl;
  if (beta < beta_low){ // use default u if L is small
    u_1 = 0;
    u_2 = 0;
    u_3 = 0;
    dL = dV(u_1, u_2, u_3, th, x, y, v, p);
  }else if (beta < 2*beta_low && u_1_prev == 0 && u_2_prev == 0 && u_3_prev == 0){ // continue using default u
    u_1 = u_1_prev;
    u_2 = u_2_prev;
    u_3 = u_3_prev;
    dL = dV(u_1, u_2, u_3, th, x, y, v, p);
  }else if (Lpure < 0.3){
    u_1 = 0;
    u_2 = dth_r/(-2.9380*v_r);
    u_3 = 0;
    dL = dV(u_1, u_2, u_3, th, x, y, v, p);
  }else{
    double phi = 0;
    double dec_rate = 0;
    if (beta > beta_low){ // Sontage formula
      dec_rate = (0.5*sqrt(a*a+beta));
      phi = (dec_rate+a)/beta;
      u_1 = -b_1*phi;
      u_2 = -b_2*phi;
      u_3 = -b_3*phi;
      u_1 = (u_1>u1_max)?u1_max:((u_1<u1_min)?u1_min:u_1);
      u_2 = (u_2>u2_max)?u2_max:((u_2<u2_min)?u2_min:u_2);
      u_3 = (u_3>u3_max)?u3_max:((u_3<u3_min)?u3_min:u_3);
    }
    dL = dV(u_1, u_2, u_3, th, x, y, v, p);

    double thresh = -0.1*L;
    double coef = 1.1;
    while (dL > thresh){ // increase decrease rate size if L is not decreasing
      if(phi == 0 || beta < beta_low)
        break;
      dec_rate = coef*dec_rate;
      phi = (dec_rate + a)/beta;
      double u_1t = -b_1*phi; double u_2t = -b_2*phi; double u_3t = -b_3*phi;
      u_1t = (u_1t>u1_max)?u1_max:((u_1t<u1_min)?u1_min:u_1t);
      u_2t = (u_2t>u2_max)?u2_max:((u_2t<u2_min)?u2_min:u_2t);
      u_3t = (u_3t>u3_max)?u3_max:((u_3t<u3_min)?u3_min:u_3t);
      // std::cout << ";" << dec_rate << ", " << phi << ":" << u_1t << ", " << u_2t << ", " << u_3t << std::endl;
      double dL_t = dV(u_1t, u_2t, u_3t, th, x, y, v, p);
      if (dL_t > dL){
        if (L > boundary){
          std::cout << "error: dL increases... V:" << L << ", Beta:" << beta << std::endl;
        }
        break;
      }
      dL = dL_t; u_1 = u_1t; u_2 = u_2t; u_3 = u_3t;
      if ((u_1 == u1_min || u_1 == u1_max) && (u_2 == u2_min || u_2 == u2_max)  && (u_3 == u3_min || u_3 == u3_max)){
        break;
      }
    }
    // if (dL > thresh)
    //   std::cout << "dL :" << dL << std::endl;
  }
  return Input(u_1, u_2, u_3, seg);
}
#endif
