#ifndef CLF_H__
#define CLF_H__

double V(double th, double x, double y, double v, double p){
  //Path Following:
  return 0.23467930931*p + 1.63456357893*th*th + 3.00993802074*x*x + 3.61402339917*y*y + 1.60970679636*v*v -1.59174813962*th*x + 0.0921162459212*th*y +0.0279626412588*th*v + 0.493507533181*x*y -0.317799799472*v*x -0.444818507944*v*y;
}

double V_pure(double th, double x, double y, double v){
  return 1.63456357893*th*th + 3.00993802074*x*x + 3.61402339917*y*y + 1.60970679636*v*v -1.59174813962*th*x + 0.0921162459212*th*y +0.0279626412588*th*v + 0.493507533181*x*y -0.317799799472*v*x -0.444818507944*v*y;
}

double dV(double u_1, double u_2, double u_3, double th, double x, double y, double v, double p){
  double v_r = 1.57;
  double dth_r = (3.1416/11.0437848)*(0.8+1.2*(3*(p-1.53)*(p-1.53)));
  double v_t = v + v_r;
  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);
  double r = (2*1.63456357893*th -1.59174813962*x + 0.0921162459212*y +0.0279626412588*v)*(-2.9380*v_t*u_2 - dth_r*(1+u_3)) + 
    (2*3.00993802074*x - 1.54782652613*th + 0.493507533181*y -0.317799799472*v)*(dth_r*(1+u_3)*y + v_t*-sin_th) + 
    (2*3.61402339917*y + 0.0921162459212*th + 0.493507533181*x -0.444818507944*v)*(dth_r*(1+u_3)*(-x) + v_t*cos_th - v_r*(1+u_3)) + 
    (2*1.60970679636*v +0.0279626412588*th -0.317799799472*x + -0.444818507944*y)*(u_1);
  double boundary = 1;
  double L = V(th, x, y, v, p);
  if( L > boundary )
    r += 0.23467930931*(1+u_3);
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
  
  double horizon = 3.06;
  int seg = (int)(rem(p_t, horizon*2)/horizon);
  if(seg != seg_prev)
    std::cout << "\n\n\nsegment " << seg << "\n\n\n" << std::endl;

  double p = rem(p_t, horizon);

  double new_x = x_t;
  double new_y = y_t;
  double new_th = th_t;
  if (seg == 1){
      new_x = -x_t;
      new_y = -y_t;
      new_th = th_t-(SP_PI);
  }
  new_th = rem((new_th+3*SP_PI), (2*SP_PI))-SP_PI;

  x_t = new_x;
  y_t = new_y;
  th_t = new_th;

  // std::cout << "state: " << th_t << ", " << x_t << ", " << y_t << ", " << v_t << ", " << p << std::endl;

  double dth_r = (3.1416/11.0437848)*(0.8+1.2*(3*(p-1.53)*(p-1.53)));
  double th_r = rem((((3.1416/11.0437848)*(0.8*p+1.2*(3.581577+(p-1.53)*(p-1.53)*(p-1.53))))+SP_PI), (2*SP_PI))-SP_PI;
  double v_r = 1.57;
  double p2 = p*p;
  double p3 = p2*p;
  double x_r = 0.0610*p2*p3   -0.4666*p3*p    +1.4088*p3   -2.0969*p2   -0.0255*p    +1.9524;
  double y_r = -0.0887*p3*p    +0.5428*p3   -1.4705*p2    +1.9585*p   -0.0256;
  double th_d = th_t - th_r;
  double v_d  = v_t  - v_r;
  double x_d  = x_t  - x_r;
  double y_d  = y_t  - y_r;
  double th = th_d;
  double v = v_d;
  double x = cos(-th_r)*x_d-sin(-th_r)*y_d;
  double y = sin(-th_r)*x_d+cos(-th_r)*y_d;
  double Lpure = V_pure(th, x, y, v);
  double L = V(th, x, y, v, p);
  std::cout << "L: " << Lpure << std::endl;

  double sin_th = (-0.1569*th*th*th+0.9977*th);
  double cos_th = (-0.4622*th*th+0.9959);

  double boundary = 1;

  double b_1 = (2*1.60970679636*v +0.0279626412588*th -0.317799799472*x + -0.444818507944*y);
  double b_2 = (2*1.63456357893*th -1.59174813962*x + 0.0921162459212*y +0.0279626412588*v)*(-2.9380*(v_t));
  double b_3 = (2*1.63456357893*th -1.59174813962*x + 0.0921162459212*y +0.0279626412588*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.00993802074*x - 1.54782652613*th + 0.493507533181*y -0.317799799472*v)*(dth_r*(1+0)*y) + 
    (2*3.61402339917*y + 0.0921162459212*th + 0.493507533181*x -0.444818507944*v)*(dth_r*(1+0)*(-x) - v_r*(1+0)) + 
    (2*1.60970679636*v +0.0279626412588*th -0.317799799472*x + -0.444818507944*y)*(0);
  if (L > boundary){
    b_3 += 0.23467930931;
  }
  double a = (2*2.31106217689*th - 1.03187637766*x - 0.521359195573*y - 0.161523642071*v)*(-2.9380*v_t*0 - dth_r*(1+0)) + 
    (2*3.23504654641*x - 1.03187637766*th + 0.825082067471*y - 0.914355165442*v)*(dth_r*(1+0)*y + v_t*-sin_th) + 
    (2*3.28654452988*y - 0.521359195573*th + 0.825082067471*x + 0.0894413885605*v)*(dth_r*(1+0)*(-x) + v_t*cos_th - v_r*(1+0)) + 
    (2*1.77745630912*v - 0.161523642071*th - 0.914355165442*x + 0.0894413885605*y)*(0);
  if (L > boundary){
    a += 0.23467930931;
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
    dL = dV(u_1, u_2, u_3, th, x, y, v, p);
  }else if (Lpure < 0.02){
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
