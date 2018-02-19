#ifndef CLF_H__
#define CLF_H__

double V(double th, double x, double y, double v, double p){
  // return 0.249999989812*p + 15.5363574309*th*th + 36.3157269731*x*x + 36.8049532397*y*y + 42.5683695983*v*v - 20.7293153886*th*x - 7.81647454596*th*y - 1.67422174172*th*v + 17.5873008649*x*y + 18.0292270703*v*x + 15.8886367948*v*y;
  return 0.308442640854*p + 14.3343025074*th*th + 5.01370493039*x*x + 4.3228973109*y*y + 2.39547903927*v*v - 5.8502319747*th*x + 0.410115549268*th*y + 1.70487162233 *th*v -0.35232265023*x*y + 1.12866155309 *v*x + 0.747137447174*v*y;
}

double V_pure(double th, double x, double y, double v){
  // return 15.5363574309*th*th + 36.3157269731*x*x + 36.8049532397*y*y + 42.5683695983*v*v - 20.7293153886*th*x - 7.81647454596*th*y - 1.67422174172*th*v + 17.5873008649*x*y + 18.0292270703*v*x + 15.8886367948*v*y;
  return 14.3343025074*th*th + 5.01370493039*x*x + 4.3228973109*y*y + 2.39547903927*v*v - 5.8502319747*th*x + 0.410115549268*th*y + 1.70487162233 *th*v -0.35232265023*x*y + 1.12866155309 *v*x + 0.747137447174*v*y;
}

double dV(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double v_r = 2;
  double dth_r = -2*(3*(p-1)*(p-1)-1);
  double v_t = v + v_r;
  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);
  // double r = (2*15.5363574309*th -20.7293153886*x - 7.81647454596 *y - 1.67422174172*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) + 
  //   (2*36.3157269731*x -20.7293153886*th +17.5873008649*y + 18.0292270703 *v)*(dth_r*(1+u_3)*y + v_t*-sin_th) + 
  //   (2*36.8049532397*y - 7.81647454596 *th +17.5873008649*x + 15.8886367948*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) + 
  //   (2*42.5683695983*v - 1.67422174172*th + 18.0292270703 *x + 15.8886367948*y)*(u_1);
  double r = (2*14.3343025074*th -5.8502319747*x + 0.410115549268 *y + 1.70487162233*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) + 
     (2*5.01370493039*x -5.8502319747*th -0.35232265023*y + 1.12866155309 *v)*(dth_r*(1+u_3)*y + v_t*-sin_th) + 
     (2*4.3228973109*y + 0.410115549268 *th -0.35232265023*x + 0.747137447174*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) + 
     (2*2.39547903927*v + 1.70487162233*th + 1.12866155309 *x + 0.747137447174*y)*(u_1);
  double boundary = 1;
  double L = V(th, x, y, v, p);
  if( L > boundary )
    // r += 0.249999989812*(1+u_3);
    r += 0.308442640854*(1+u_3);
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

  double dth_r = -2*(3*(p-1)*(p-1)-1);
  double th_r = -2*((p-1)*(p-1)*(p-1)-(p-1)) - (-2*((0-1)*(0-1)*(0-1)-(0-1))) + SP_PI/2;
  double v_r = 2;
  double p3 = p*p*p;
  double p4 = p*p3;
  double x_r = 0.5482*p3*p4 -3.8375*p3*p3 + 9.9839*p4*p  -11.5446*p4  +  5.1725*p3 -0.0394*p*p   -1.9892*p  + 1.4996;
  double y_r = -0.1209*p3*p3 +   0.7252*p4*p   -0.7209*p4   -1.9509*p3  + 2.8299*p*p +   0.1758*p   -0.0056;
  double th_d = th_t - th_r;
  double v_d  = v_t  - v_r;
  double x_d  = x_t  - x_r;
  double y_d  = y_t  - y_r;
  double th = th_d;
  double v = v_d;
  double x = cos(-th_r)*x_d-sin(-th_r)*y_d;
  double y = sin(-th_r)*x_d+cos(-th_r)*y_d;

  double L = V(th, x, y, v, p);
  std::cout << "L: " << L << std::endl;

  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);

  double boundary = 1;

  // double b_1 = (2*42.5683695983*v - 1.67422174172*th + 18.0292270703 *x + 15.8886367948*y);
  // double b_2 = (2*15.5363574309*th -20.7293153886*x - 7.81647454596 *y - 1.67422174172*v)*(-2.9380*v_t);
  // double b_3 = (2*15.5363574309*th -20.7293153886*x - 7.81647454596 *y - 1.67422174172*v)*(-2.9380*v_t*0 - dth_r) + 
  //   (2*36.3157269731*x -20.7293153886*th +17.5873008649*y + 18.0292270703 *v)*(dth_r*y) + 
  //   (2*36.8049532397*y - 7.81647454596 *th +17.5873008649*x + 15.8886367948*v)*(dth_r*(-x) - v_r) + 
  //   (2*42.5683695983*v - 1.67422174172*th + 18.0292270703 *x + 15.8886367948*y)*(0);
  double  b_1 = (2*2.39547903927*v + 1.70487162233*th + 1.12866155309 *x + 0.747137447174*y);
  double  b_2 = (2*14.3343025074*th -5.8502319747*x + 0.410115549268 *y + 1.70487162233*v)*(-2.9380*(v_t));
  double  b_3 = (2*14.3343025074*th -5.8502319747*x + 0.410115549268 *y + 1.70487162233*v)*(-2.9380*v_t*0 - dth_r) + 
     (2*5.01370493039*x -5.8502319747*th -0.35232265023*y + 1.12866155309 *v)*(dth_r*y ) + 
     (2*4.3228973109*y + 0.410115549268 *th -0.35232265023*x + 0.747137447174*v)*(dth_r*(-x) - v_r) + 
     (2*2.39547903927*v + 1.70487162233*th + 1.12866155309 *x + 0.747137447174*y)*(0);
  if (L > boundary){
    // b_3 += 0.249999989812;
    b_3 += 0.308442640854;
  }
  // double a = (2*15.5363574309*th -20.7293153886*x - 7.81647454596 *y - 1.67422174172*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
  //   (2*36.3157269731*x -20.7293153886*th +17.5873008649*y + 18.0292270703 *v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
  //   (2*36.8049532397*y - 7.81647454596 *th +17.5873008649*x + 15.8886367948*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
  //   (2*42.5683695983*v - 1.67422174172*th + 18.0292270703 *x + 15.8886367948*y)*(0);
  double a = (2*14.3343025074*th -5.8502319747*x + 0.410115549268 *y + 1.70487162233*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
     (2*5.01370493039*x -5.8502319747*th -0.35232265023*y + 1.12866155309 *v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
     (2*4.3228973109*y + 0.410115549268 *th -0.35232265023*x + 0.747137447174*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
     (2*2.39547903927*v + 1.70487162233*th + 1.12866155309 *x + 0.747137447174*y)*(0);
  if (L > boundary){
    // a += 0.249999989812;
    a += 0.308442640854;
  }
  double beta = b_1*b_1 + b_2*b_2 + b_3*b_3;
  
  double u1_max = 4;
  double u1_min = -4;
  double u2_max = 1;
  double u2_min = -1;
  double u3_max = 9;
  double u3_min = -0.9;

  double beta_low = 0.1/2;
  double u_1 = 0;
  double u_2 = 0;
  double u_3 = 0;

  double dL = 0;
  double Lpure = V_pure(th, x, y, v);
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
  }else if (Lpure < 0.7){
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
