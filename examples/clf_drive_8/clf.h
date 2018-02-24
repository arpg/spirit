#ifndef CLF_H__
#define CLF_H__

double V2(double th, double x, double y, double v, double p){
  //Path Following:
  return 0.173534449404*p + 2.50517610404*th*th + 3.38730383709*x*x + 2.6312083175*y*y + 1.96652241333*v*v -1.61453242671*th*x -1.15385410843*th*y + 0.346919248873*th*v -1.39918220993*x*y + 0.938702983825*v*x -0.239203587927*v*y;
}

double V2_pure(double th, double x, double y, double v){
  return 2.50517610404*th*th + 3.38730383709*x*x + 2.6312083175*y*y + 1.96652241333*v*v -1.61453242671*th*x -1.15385410843*th*y + 0.346919248873*th*v -1.39918220993*x*y + 0.938702983825*v*x -0.239203587927*v*y;
}

double dV2(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double v_r = 1.57 + 0.1*(p-2)*(p-2) - 0.1*(0-2)*(0-2);
  double dth_r = -0.5*(p-2);
  double v_t = v + v_r;
  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);
  double r = (2*2.50517610404*th -1.61453242671*x -1.15385410843*y + 0.0205482224817*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) +
    (2*3.38730383709*x -1.61453242671*th -1.39918220993*y + 0.346919248873*v)*(dth_r*(1+u_3)*y + v_t*-sin_th) +
    (2*2.6312083175*y -1.15385410843*th -1.39918220993*x -0.239203587927*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) +
    (2*1.96652241333*v + 0.0205482224817*th + 0.346919248873*x -0.239203587927*y)*(u_1);
  double boundary = 1;
  double L = V2(th, x, y, v, p);
  if( L > boundary )
    r += 0.173534449404*(1+u_3);
  return r;
}

double V1(double th, double x, double y, double v, double p){
  //Path Following:
  return 0.132714936796*p + 1.19737722847*th*th + 3.60432607705 *x*x + 3.3823206587*y*y + 1.56919229828*v*v -1.34777755092*th*x +1.40137390139*th*y -1.41379464541*th*v + 0.911461344612*x*y +0.278337549213*v*x -0.714059696346*v*y;
}

double V1_pure(double th, double x, double y, double v){
  return 1.19737722847*th*th + 3.60432607705 *x*x + 3.3823206587*y*y + 1.56919229828*v*v -1.34777755092*th*x +1.40137390139*th*y -1.41379464541*th*v + 0.911461344612*x*y +0.278337549213*v*x -0.714059696346*v*y;
}

double dV1(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double v_r = 1.57;
  double dth_r = 1.047197551;
  double v_t = v + v_r;
  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);
  double r = (2*1.19737722847*th -1.34777755092*x +1.40137390139*y -1.41379464541*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) + 
    (2*3.60432607705 *x -1.34777755092*th + 0.911461344612*y +0.278337549213*v)*(dth_r*(1+u_3)*y + v_t*-sin_th) + 
    (2*3.3823206587*y +1.40137390139*th + 0.911461344612*x -0.714059696346*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) + 
    (2*1.56919229828*v -1.41379464541*th +0.278337549213*x -0.714059696346*y)*(u_1);
  double boundary = 1;
  double L = V1(th, x, y, v, p);
  if( L > boundary )
    r += 0.132714936796*(1+u_3);
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

Input K2(double th_t, double x_t, double y_t, double v_t, double p_t, int seg_prev, double u_1_prev, double u_2_prev, double u_3_prev){
  
  // double horizon = 1.5;
  // int seg = (int)(rem(p_t, horizon*4)/horizon);
  // if(seg != seg_prev)
  //   std::cout << "\n\n\nsegment " << seg << "\n\n\n" << std::endl;
  int seg = 0;
  double p = p_t;//rem(p_t, horizon);

  // double new_x = x_t;
  // double new_y = y_t;
  // double new_th = th_t;
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
  // new_th = rem((new_th+SP_PI), (2*SP_PI))-SP_PI;

  // x_t = new_x;
  // y_t = new_y;
  // th_t = new_th;

  // std::cout << "state: " << th_t << ", " << x_t << ", " << y_t << ", " << v_t << ", " << p << std::endl;

  double dth_r = -0.5*(p_t-2);
  double th_r = rem(((-0.25*(p_t-2)*(p_t-2) - -0.25*(0-2)*(0-2))+SP_PI), (2*SP_PI))-SP_PI;
  double v_r = 1.57 + 0.1*(p_t-2)*(p_t-2) - 0.1*(0-2)*(0-2);
  double p2 = p*p;
  double p3 = p2*p;
  double x_r = 0.0087*p3*p2   -0.0866*p3*p    +0.3719*p3   -0.8466*p2    +0.0155*p    +1.4990;
  double y_r = 0.0904*p3 -0.5427*p2    +1.7289*p   -0.0186;
  double th_d = th_t - th_r;
  double v_d  = v_t  - v_r;
  double x_d  = x_t  - x_r;
  double y_d  = y_t  - y_r;
  double th = th_d;
  double v = v_d;
  double x = cos(-th_r)*x_d-sin(-th_r)*y_d;
  double y = sin(-th_r)*x_d+cos(-th_r)*y_d;
  double Lpure = V2_pure(th, x, y, v);
  double L = V2(th, x, y, v, p);
  std::cout << "L: " << Lpure << std::endl;

  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);

  double boundary = 1;

  double b_1 = (2*1.96652241333*v + 0.0205482224817*th + 0.346919248873*x -0.239203587927*y);
  double b_2 = (2*2.50517610404*th -1.61453242671*x -1.15385410843*y + 0.0205482224817*v)*(-2.9380*(v_t));
  double b_3 = (2*2.50517610404*th -1.61453242671*x -1.15385410843*y + 0.0205482224817*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.38730383709*x -1.61453242671*th -1.39918220993*y + 0.346919248873*v)*(dth_r*(1+0)*y) + 
    (2*2.6312083175*y -1.15385410843*th -1.39918220993*x -0.239203587927*v)*(dth_r*(1+0)*(-x) - v_r*(1+0)) + 
    (2*1.96652241333*v + 0.0205482224817*th + 0.346919248873*x -0.239203587927*y)*(0);
  if (L > boundary){
    b_3 += 0.173534449404;
  }
  double a = (2*2.50517610404*th -1.61453242671*x -1.15385410843*y + 0.0205482224817*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.38730383709*x -1.61453242671*th -1.39918220993*y + 0.346919248873*v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
    (2*2.6312083175*y -1.15385410843*th -1.39918220993*x -0.239203587927*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
    (2*1.96652241333*v + 0.0205482224817*th + 0.346919248873*x -0.239203587927*y)*(0);
  if (L > boundary){
    a += 0.173534449404;
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
  
  if (Lpure < 0.04 && u_1_prev == 0 && u_2_prev == dth_r/(-2.9380*v_r) && u_3_prev == 0){ // continue using default u
    u_1 = u_1_prev;
    u_2 = u_2_prev;
    u_3 = u_3_prev;
    dL = dV2(u_1, u_2, u_3, th, x, y, v, p);
  }else if (Lpure < 0.02){
    u_1 = 0;
    u_2 = dth_r/(-2.9380*v_r);
    u_3 = 0;
    dL = dV2(u_1, u_2, u_3, th, x, y, v, p);
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
    dL = dV2(u_1, u_2, u_3, th, x, y, v, p);

    double thresh = -0.0*L;
    double e_q = 0.2;
    if (L > boundary)
      thresh -= e_q;
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
      double dL_t = dV2(u_1t, u_2t, u_3t, th, x, y, v, p);
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

Input K1(double th_t, double x_t, double y_t, double v_t, double p_t, int seg_prev, double u_1_prev, double u_2_prev, double u_3_prev){
  
  double horizon = 1.5;
  int seg = (int)(rem(p_t, horizon*4)/horizon);
  if(seg != seg_prev)
    std::cout << "\n\n\nsegment " << seg << "\n\n\n" << std::endl;

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
  new_th = rem((new_th+3*SP_PI), (2*SP_PI))-SP_PI;

  x_t = new_x;
  y_t = new_y;
  th_t = new_th;

  // std::cout << "state: " << th_t << ", " << x_t << ", " << y_t << ", " << v_t << ", " << p << std::endl;

  double dth_r = 1.047197551;
  double th_r = rem((dth_r*p+SP_PI), (2*SP_PI))-SP_PI;
  double v_r = 1.57;
  double x_r = 1.5*cos(th_r);
  double y_r = 1.5*sin(th_r);
  double th_d = th_t - th_r;
  double v_d  = v_t  - v_r;
  double x_d  = x_t  - x_r;
  double y_d  = y_t  - y_r;
  double th = th_d;
  double v = v_d;
  double x = cos(-th_r)*x_d-sin(-th_r)*y_d;
  double y = sin(-th_r)*x_d+cos(-th_r)*y_d;
  double Lpure = V1_pure(th, x, y, v);
  double L = V1(th, x, y, v, p);
  std::cout << "L: " << Lpure << std::endl;

  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);

  double boundary = 1;

  double b_1 = (2*2.26798439113*v - 0.574485322443*th - 1.25261170684*x + 0.108793361414*y);
  double b_2 = (2*2.2684832219*th - 1.54782652613*x + 0.787553933319*y - 0.574485322443*v)*(-2.9380*(v_t));
  double b_3 = (2*1.19737722847*th -1.34777755092*x +1.40137390139*y -1.41379464541*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.60432607705 *x -1.34777755092*th + 0.911461344612*y +0.278337549213*v)*(dth_r*(1+0)*y) + 
    (2*3.3823206587*y +1.40137390139*th + 0.911461344612*x -0.714059696346*v)*(dth_r*(1+0)*(-x) - v_r*(1+0)) + 
    (2*1.56919229828*v -1.41379464541*th +0.278337549213*x -0.714059696346*y)*(0);
  if (L > boundary){
    b_3 += 0.132714936796;
  }
  double a = (2*1.19737722847*th -1.34777755092*x +1.40137390139*y -1.41379464541*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.60432607705 *x -1.34777755092*th + 0.911461344612*y +0.278337549213*v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
    (2*3.3823206587*y +1.40137390139*th + 0.911461344612*x -0.714059696346*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
    (2*1.56919229828*v -1.41379464541*th +0.278337549213*x -0.714059696346*y)*(0);
  if (L > boundary){
    a += 0.132714936796;
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
  
  if (Lpure < 0.04 && u_1_prev == 0 && u_2_prev == dth_r/(-2.9380*v_r) && u_3_prev == 0){ // continue using default u
    u_1 = u_1_prev;
    u_2 = u_2_prev;
    u_3 = u_3_prev;
    dL = dV1(u_1, u_2, u_3, th, x, y, v, p);
  }else if (Lpure < 0.02){
    u_1 = 0;
    u_2 = dth_r/(-2.9380*v_r);
    u_3 = 0;
    dL = dV1(u_1, u_2, u_3, th, x, y, v, p);
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
    dL = dV1(u_1, u_2, u_3, th, x, y, v, p);

    double thresh = -0.0*L;
    double e_q = 0.3;
    if (L > boundary)
      thresh -= e_q;
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
      double dL_t = dV1(u_1t, u_2t, u_3t, th, x, y, v, p);
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

Input K(double th_t, double x_t, double y_t, double v_t, double p_t, int seg_prev, double u_1_prev, double u_2_prev, double u_3_prev){
  p_t = rem(p_t, 14);
  int seg = floor(p_t);
  std::cout << "seg:" << seg << std::endl;
  if(seg < 3){
    x_t = -x_t;
    y_t = -y_t;
    th_t = th_t - SP_PI;
    Input res = K1(th_t,x_t,y_t,v_t, p_t, seg_prev, u_1_prev, u_2_prev, u_3_prev);
    return res;
  }else if(seg < 7){
    th_t = rem(th_t+2*SP_PI, 2*SP_PI);
    p_t = p_t - 3;
    Input res = K2(th_t,x_t,y_t,v_t, p_t, seg_prev, u_1_prev, u_2_prev, u_3_prev);
    return res;
  }else if(seg < 10){
    x_t = -x_t;
    y_t = y_t - 4;
    p_t = p_t - 7;
    th_t = -th_t;
    Input res = K1(th_t,x_t,y_t,v_t, p_t, seg_prev, u_1_prev, u_2_prev, u_3_prev);
    res.u_2 = -res.u_2;
    return res;
  }else{
    p_t = p_t - 10;
    y_t = 4 - y_t;
    th_t = -th_t + SP_PI;
    Input res = K2(th_t,x_t,y_t,v_t, p_t, seg_prev, u_1_prev, u_2_prev, u_3_prev);
    res.u_2 = -res.u_2;
    return res;
  }
}

#endif
