#include <eigen3/Eigen/Eigen>
#include <spirit/spGeneralTools.h>

int main(int argc, char** argv) {


  double vx_i = 0.1;
  double vy_i = 0.1;
  double vw_i = 0.1;
  double w0_i = 0.1;
  double w1_i = 0.1;
  double w2_i = 0.1;
  double w3_i = 0.1;
  double chi_i = 0.1;
  double sigma_i = 0.1;

  double Ix = 0.1;
  double Iy = 0.1;
  double Iw = 0.1;
  double It = 0.1;
  double mu_n_p0 = 0.1;
  double mu_n_p1 = 0.1;
  double mu_n_p2 = 0.1;
  double mu_n_p3 = 0.1;
  double mu_t_p0 = 0.1;
  double mu_t_p1 = 0.1;
  double mu_t_p2 = 0.1;
  double mu_t_p3 = 0.1;
  double str_del = 0.1;
  double k_mot = 0.1;

  double str_in = 0.1;
  double Curr_in = 0.1;

  double r = 0.1;
  double theta = 0.785;
  double d = 0.2;
  double m=12;

  double w0 = w0_i;
  double w1 = w1_i;
  double w2 = w2_i;
  double w3 = w3_i;
  double sigma = sigma_i;

  double k0 = (r*w0 / vx_i*cos(sigma))-1;
  double k1 = (r*w1 / vx_i)-1;
  double k2 = (r*w2 / vx_i)-1;
  double k3 = (r*w3 / vx_i*cos(sigma))-1;

  double beta = atan2(vy_i,vx_i);
  double beta0 = beta - sigma;
  double beta1 = beta;
  double beta2 = beta;
  double beta3 = beta - sigma;

  spTimestamp t0 = spGeneralTools::Tick();
  double sum=0;
for(int ii=0; ii<400; ii++){
  Eigen::VectorXd x_d(11);
  x_d(0) = ((mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1-vx_i+(r*w2_i)/It)-(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1+vx_i-(r*w1_i)/It)-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-1.57E2/2.0E2)*(1.0/5.0)+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i)/It)+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+1.57E2/2.0E2)*(1.0/5.0)-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+(r*w3_i)/It)+m*vw_i*vy_i-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vw_i*sin(sigma_i-1.57E2/2.0E2)*(1.0/5.0)-vy_i*cos(sigma_i)+vx_i*sin(sigma_i))+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vw_i*sin(sigma_i+1.57E2/2.0E2)*(1.0/5.0)+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)))/Ix;
  x_d(1) = ((mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210732E-1-vy_i)+(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210732E-1-cos(sigma_i)+vx_i*sin(sigma_i))-cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vw_i*sin(sigma_i+1.57E2/2.0E2)*(1.0/5.0)+vy_i*cos(sigma_i)-vx_i*sin(sigma_i))-vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i-1.57E2/2.0E2)*(1.0/5.0)+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i)/It)+vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i+1.57E2/2.0E2)*(1.0/5.0)-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+(r*w3_i)/It)+vy_i*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(-cos(sigma_i)+vw_i*sin(sigma_i-1.57E2/2.0E2)*(1.0/5.0)+vx_i*sin(sigma_i))-(Iw*m*vw_i*vx_i)/Ix)/Iy;
  x_d(2) = -((mu_n_p0*1.4147765383344E-1+(mu_n_p1*1.4147765383344E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1-vx_i+(r*w2_i)/It)+(mu_t_p0*1.413650362210732E-1+(mu_t_p1*1.413650362210732E-1)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210732E-1-vy_i)*2.0+(mu_n_p0*1.4147765383344E-1+(mu_n_p1*1.4147765383344E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1+vx_i-(r*w1_i)/It)+sin(sigma_i-1.57E2/2.0E2)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vw_i*sin(sigma_i-1.57E2/2.0E2)*(1.0/5.0)-vy_i*cos(sigma_i)+vx_i*sin(sigma_i))*(1.0/5.0)+sin(sigma_i+1.57E2/2.0E2)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)),2.0)))*(vw_i*sin(sigma_i+1.57E2/2.0E2)*(1.0/5.0)+vy_i*cos(sigma_i)-vx_i*sin(sigma_i))*(1.0/5.0)+cos(sigma_i-1.57E2/2.0E2)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-1.57E2/2.0E2)*(1.0/5.0)+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i)/It)*(1.0/5.0)+cos(sigma_i+1.57E2/2.0E2)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+1.57E2/2.0E2)*(1.0/5.0)-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+(r*w3_i)/It)*(1.0/5.0))/Iw;
  x_d(3) = Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-1.57E2/2.0E2)*(1.0/5.0)+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i)/It);
  x_d(4) = Curr_in*k_mot+r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1+vx_i-(r*w1_i)/It);
  x_d(5) = Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*(vw_i*1.4147765383344E-1-vx_i+(r*w2_i)/It);
  x_d(6) = Curr_in*k_mot-r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+1.57E2/2.0E2)*(1.0/5.0)-vx_i*cos(sigma_i)-vy_i*sin(sigma_i)+(r*w3_i)/It);
  x_d(7) = cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
  x_d(8) = sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
  x_d(9) = vw_i;
  x_d(10) = -(sigma_i-str_in)/str_del;

  Eigen::MatrixXd DfDp(11,14);
  DfDp = Eigen::MatrixXd::Zero(11,14);
  DfDp(0,0) = 1.0/(Ix*Ix)*((mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*-1.0+(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.0+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+m*vw_i*vy_i+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It))*-1.0;
  DfDp(0,3) = (1.0/(It*It)*r*w1_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*-1.0-1.0/(It*It)*r*w2_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*1.0-1.0/(It*It)*r*w0_i*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-1.0/(It*It)*r*w3_i*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0)/Ix;
  DfDp(0,4) = (vx_i*-2.0-cos(sigma_i)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+cos(sigma_i)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)+(r*w1_i)/It+(r*w2_i)/It)/Ix;
  DfDp(0,5) = ((vw_i*-1.414776538334479E-1-vx_i*1.0+(r*w1_i*1.0)/It)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))+(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i*1.0)/It)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))+(cos(sigma_i)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0))-(cos(sigma_i)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0)/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))/Ix;
  DfDp(0,6) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)-mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*1.0+mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)-mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/Ix;
  DfDp(0,7) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*pow((r*w1_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)-mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*pow((r*w2_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*1.0+mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)-mu_n_p1*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/Ix;
  DfDp(0,8) = (sin(sigma_i)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*-1.0+sin(sigma_i)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0)/Ix;
  DfDp(0,9) = ((sin(sigma_i)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*-1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))+(sin(sigma_i)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Ix;
  DfDp(0,10) = (mu_t_p1*sin(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))-mu_t_p1*sin(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0)/Ix;
  DfDp(0,11) = (mu_t_p1*sin(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))-mu_t_p1*sin(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0)/Ix;
  DfDp(1,0) = (Iw*1.0/(Ix*Ix)*m*vw_i*vx_i)/Iy;
  DfDp(1,1) = 1.0/(Iy*Iy)*((mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210603E-1-vy_i*1.0)+(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))-cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0+vy_i*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))-vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)-(Iw*m*vw_i*vx_i*1.0)/Ix)*-1.0;
  DfDp(1,2) = (m*vw_i*vx_i*-1.0)/(Ix*Iy);
  DfDp(1,3) = (1.0/(It*It)*r*vw_i*w0_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0-1.0/(It*It)*r*vw_i*w3_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0)/Iy;
  DfDp(1,4) = (vw_i*sin(sigma_i)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*-1.0+vw_i*sin(sigma_i)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/Iy;
  DfDp(1,5) = ((vw_i*sin(sigma_i)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*-1.0)/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))+(vw_i*sin(sigma_i)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))/Iy;
  DfDp(1,6) = (mu_n_p1*vw_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)-mu_n_p1*vw_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/Iy;
  DfDp(1,7) = (mu_n_p1*vw_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)-mu_n_p1*vw_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0)/Iy;
  DfDp(1,8) = (vw_i*2.827300724421207E-1-vy_i*1.0-cos(sigma_i)*1.0-cos(sigma_i)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0+vx_i*sin(sigma_i)+vy_i*cos(sigma_i)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i)))/Iy;
  DfDp(1,9) = ((vw_i*1.413650362210603E-1-vy_i*1.0)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(cos(sigma_i)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))+(vy_i*cos(sigma_i)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i)))/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Iy;
  DfDp(1,10) = (mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*-1.0-mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.0+mu_t_p1*cos(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0-mu_t_p1*vy_i*cos(sigma_i)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*1.0)/Iy;
  DfDp(1,11) = (mu_t_p1*cos(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0-mu_t_p1*vy_i*cos(sigma_i)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*1.0-(mu_t_p1*1.0/(vx_i*vx_i)*(vy_i*vy_i)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*1.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-(mu_t_p1*1.0/(vx_i*vx_i)*(vy_i*vy_i)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Iy;
  DfDp(2,2) = 1.0/(Iw*Iw)*((mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)+(mu_t_p0*1.413650362210603E-1+(mu_t_p1*1.413650362210603E-1)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(vw_i*1.413650362210603E-1-vy_i*1.0)*2.0+(mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)+cos(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1+cos(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1+sin(sigma_i-7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1+sin(sigma_i+7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1);
  DfDp(2,3) = (1.0/(It*It)*r*w1_i*(mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*-1.0+1.0/(It*It)*r*w2_i*(mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*1.0-1.0/(It*It)*r*w0_i*cos(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1+1.0/(It*It)*r*w3_i*cos(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1)/Iw;
  DfDp(2,4) = (vw_i*-4.003185306842738E-2-cos(sigma_i+7.849999999999682E-1)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1-cos(sigma_i-7.849999999999682E-1)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1+(r*w1_i*1.414776538334479E-1)/It-(r*w2_i*1.414776538334479E-1)/It)/Iw;
  DfDp(2,5) = ((vw_i*-2.001592653421692E-2-vx_i*1.414776538334479E-1+(r*w1_i*1.414776538334479E-1)/It)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))-(vw_i*2.001592653421692E-2-vx_i*1.414776538334479E-1+(r*w2_i*1.414776538334479E-1)/It)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))-(cos(sigma_i-7.849999999999682E-1)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1)/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))-(cos(sigma_i+7.849999999999682E-1)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1)/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))/Iw;
  DfDp(2,6) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*1.414776538334479E-1+mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*1.414776538334479E-1+mu_n_p1*cos(sigma_i-7.849999999999682E-1)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1+mu_n_p1*cos(sigma_i+7.849999999999682E-1)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1)/Iw;
  DfDp(2,7) = (mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*pow((r*w1_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*1.414776538334479E-1+mu_n_p1*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*pow((r*w2_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*1.414776538334479E-1+mu_n_p1*cos(sigma_i-7.849999999999682E-1)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1+mu_n_p1*cos(sigma_i+7.849999999999682E-1)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1)/Iw;
  DfDp(2,8) = (vw_i*-3.996814693157091E-2+vy_i*2.827300724421207E-1-sin(sigma_i-7.849999999999682E-1)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1-sin(sigma_i+7.849999999999682E-1)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1)/Iw;
  DfDp(2,9) = ((vw_i*-3.99681469315634E-2+vy_i*2.827300724421207E-1)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(sin(sigma_i-7.849999999999682E-1)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0))-(sin(sigma_i+7.849999999999682E-1)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1)/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))/Iw;
  DfDp(2,10) = (mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*2.827300724421207E-1+mu_t_p1*sin(sigma_i-7.849999999999682E-1)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1+mu_t_p1*sin(sigma_i+7.849999999999682E-1)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1)/Iw;
  DfDp(2,11) = (mu_t_p1*sin(sigma_i-7.849999999999682E-1)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1+mu_t_p1*sin(sigma_i+7.849999999999682E-1)*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1+(mu_t_p1*1.0/(vx_i*vx_i)*(vy_i*vy_i)*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*2.827300724421207E-1)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Iw;
  DfDp(3,3) = 1.0/(It*It)*(r*r)*w0_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)));
  DfDp(3,4) = r*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It);
  DfDp(3,5) = (r*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It))/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0));
  DfDp(3,6) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*-1.0;
  DfDp(3,7) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*-1.0;
  DfDp(3,13) = Curr_in;
  DfDp(4,3) = 1.0/(It*It)*(r*r)*w1_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)));
  DfDp(4,4) = r*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It);
  DfDp(4,5) = (r*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It))/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0));
  DfDp(4,6) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*-1.0;
  DfDp(4,7) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*pow((r*w1_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*-1.0;
  DfDp(4,13) = Curr_in;
  DfDp(5,3) = 1.0/(It*It)*(r*r)*w2_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)));
  DfDp(5,4) = r*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*-1.0;
  DfDp(5,5) = (r*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*-1.0)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0));
  DfDp(5,6) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It);
  DfDp(5,7) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*pow((r*w2_i)/vx_i-1.0,2.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It);
  DfDp(5,13) = Curr_in;
  DfDp(6,3) = 1.0/(It*It)*(r*r)*w3_i*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)));
  DfDp(6,4) = r*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*-1.0;
  DfDp(6,5) = (r*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*-1.0)/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0));
  DfDp(6,6) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It);
  DfDp(6,7) = mu_n_p1*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It);
  DfDp(6,13) = Curr_in;
  DfDp(10,12) = 1.0/(str_del*str_del)*(sigma_i-str_in*1.0);

  Eigen::MatrixXd DfDx(11,11);
  DfDx = Eigen::MatrixXd::Zero(11,11);

  DfDx(0,0) = (mu_n_p0*-2.0-pow(cos(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(cos(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-pow(sin(sigma_i),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-(mu_n_p1*1.0)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))-(mu_n_p1*1.0)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))+mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.0-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.0-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w0_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0+mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w3_i*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))/Ix;
  DfDx(0,1) = (m*vw_i+cos(sigma_i)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*2.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-(mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+(mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Ix;
  DfDx(0,2) = (m*vy_i-(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))-cos(sigma_i-7.849999999999682E-1)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1+cos(sigma_i+7.849999999999682E-1)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1-sin(sigma_i-7.849999999999682E-1)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1+sin(sigma_i+7.849999999999682E-1)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1)/Ix;
  DfDx(0,3) = ((r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))))/It+(mu_n_p1*mu_n_p3*r*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i)/Ix;
  DfDx(0,4) = ((r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))))/It+(mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.0)/vx_i)/Ix;
  DfDx(0,5) = ((r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))))/It-(mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.0)/vx_i)/Ix;
  DfDx(0,6) = ((r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0))))/It-(mu_n_p1*mu_n_p3*r*pow(cos(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i)/Ix;
  DfDx(0,10) = (cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*-1.0+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.0-cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i))*1.0-sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*-1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i))*1.0-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.0+mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*2.0-mu_t_p1*mu_t_p3*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0-(mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i+(mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i)/Ix;
  DfDx(1,0) = (sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))+cos(sigma_i)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))+mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i*vx_i)*(vy_i*vy_i)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))-vw_i*cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-vw_i*cos(sigma_i)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-(Iw*m*vw_i*1.0)/Ix+vy_i*cos(sigma_i)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))+mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*((mu_t_p3*1.0/(vx_i*vx_i*vx_i)*(vy_i*vy_i)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)-(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i)*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_n_p1*mu_n_p3*r*vw_i*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0+mu_n_p1*mu_n_p3*r*vw_i*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/Iy;
  DfDx(1,1) = (mu_t_p0*-1.0-pow(cos(sigma_i),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.0-(mu_t_p1*1.0)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*1.0-vw_i*pow(sin(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-vw_i*pow(sin(sigma_i),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.0-mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i)*vy_i*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*1.0-mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i)*vy_i*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*(vw_i*1.413650362210603E-1-cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.0-(mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))+(mu_t_p1*mu_t_p3*vy_i*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*2.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Iy;
  DfDx(1,2) = (mu_t_p0*2.827300724421207E-1+(mu_t_p1*2.827300724421207E-1)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)-sin(sigma_i+7.849999999999682E-1)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1-(Iw*m*vx_i*1.0)/Ix+vy_i*sin(sigma_i-7.849999999999682E-1)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1)/Iy;
  DfDx(1,3) = ((r*vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0))))/It+(mu_n_p1*mu_n_p3*r*vw_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i)/Iy;
  DfDx(1,6) = ((r*vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0))))/It-(mu_n_p1*mu_n_p3*r*vw_i*cos(sigma_i)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i)/Iy;
  DfDx(1,10) = ((mu_t_p0+mu_t_p1/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))*(sin(sigma_i)+vx_i*cos(sigma_i))+sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)+cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*-1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i))-vy_i*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*1.0+vy_i*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(sin(sigma_i)+vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i))-vw_i*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.0+vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))-vw_i*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.0+vw_i*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)+mu_t_p1*mu_t_p3*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*2.0-mu_t_p1*mu_t_p3*vy_i*cos(sigma_i)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(cos(sigma_i)*-1.0+vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*sin(sigma_i))*2.0-(mu_n_p1*mu_n_p3*r*vw_i*w0_i*pow(sin(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i+(mu_n_p1*mu_n_p3*r*vw_i*w3_i*pow(sin(sigma_i),2.0)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i)/Iy;
  DfDx(2,0) = ((mu_n_p1*-1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0))-cos(sigma_i-7.849999999999682E-1)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1+cos(sigma_i+7.849999999999682E-1)*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1-sin(sigma_i-7.849999999999682E-1)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1+sin(sigma_i+7.849999999999682E-1)*sin(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1-mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*((mu_t_p3*1.0/(vx_i*vx_i*vx_i)*(vy_i*vy_i)*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*2.827300724421207E-1-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.829553076668958E-1-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.829553076668958E-1+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*sin(sigma_i-7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*3.999999999999773E-1)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)+(mu_t_p1*mu_t_p3*1.0/(vx_i*vx_i)*vy_i*sin(sigma_i+7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*3.999999999999773E-1)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i-7.849999999999682E-1)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*3.999999999999773E-1-mu_n_p1*mu_n_p3*r*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i+7.849999999999682E-1)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*3.999999999999773E-1)/Iw;
  DfDx(2,1) = (mu_t_p0*2.827300724421207E-1+(mu_t_p1*2.827300724421207E-1)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-cos(sigma_i-7.849999999999682E-1)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1+cos(sigma_i+7.849999999999682E-1)*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1+sin(sigma_i-7.849999999999682E-1)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1-sin(sigma_i+7.849999999999682E-1)*cos(sigma_i)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*1.999999999999886E-1+mu_t_p1*1.0/pow(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0),2.0)*((mu_t_p3*1.0/(vx_i*vx_i)*vy_i*2.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)-mu_t_p3*1.0/(vx_i*vx_i*vx_i*vx_i)*(vy_i*vy_i*vy_i)*1.0/pow(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0,2.0)*2.0)*(vw_i*1.413650362210603E-1-vy_i*1.0)*2.827300724421207E-1-(mu_t_p1*mu_t_p3*sin(sigma_i-7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*3.999999999999773E-1)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-(mu_t_p1*mu_t_p3*sin(sigma_i+7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*3.999999999999773E-1)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0)))/Iw;
  DfDx(2,2) = (mu_n_p0*-4.003185306842738E-2-mu_t_p0*3.996814693157091E-2-pow(sin(sigma_i-7.849999999999682E-1),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*3.999999999999915E-2-pow(sin(sigma_i+7.849999999999682E-1),2.0)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*3.999999999999915E-2-(mu_t_p1*3.996814693157091E-2)/(mu_t_p2+(mu_t_p3*1.0/(vx_i*vx_i)*(vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0))-pow(cos(sigma_i-7.849999999999682E-1),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*3.999999999999915E-2-pow(cos(sigma_i+7.849999999999682E-1),2.0)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*3.999999999999915E-2-(mu_n_p1*2.001592653421369E-2)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))-(mu_n_p1*2.001592653421369E-2)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))/Iw;
  DfDx(2,3) = ((r*cos(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1)/It+(mu_n_p1*mu_n_p3*r*cos(sigma_i-7.849999999999682E-1)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*3.999999999999773E-1)/vx_i)/Iw;
  DfDx(2,4) = ((r*(mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0))))/It+(mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.829553076668958E-1)/vx_i)/Iw;
  DfDx(2,5) = ((r*(mu_n_p0*1.414776538334479E-1+(mu_n_p1*1.414776538334479E-1)/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*-1.0)/It+(mu_n_p1*mu_n_p3*r*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.829553076668958E-1)/vx_i)/Iw;
  DfDx(2,6) = ((r*cos(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.999999999999886E-1)/It+(mu_n_p1*mu_n_p3*r*cos(sigma_i+7.849999999999682E-1)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*3.999999999999773E-1)/vx_i)/Iw;
  DfDx(2,10) = (sin(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*1.999999999999886E-1+cos(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1+cos(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1-sin(sigma_i-7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i))*1.999999999999886E-1+sin(sigma_i+7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*-1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i))*1.999999999999886E-1+sin(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*1.999999999999886E-1-cos(sigma_i-7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*1.999999999999886E-1-cos(sigma_i+7.849999999999682E-1)*(mu_t_p0+mu_t_p1/(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*1.999999999999886E-1+mu_t_p1*mu_t_p3*sin(sigma_i-7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*3.999999999999773E-1+mu_t_p1*mu_t_p3*sin(sigma_i+7.849999999999682E-1)*cos(sigma_i-atan(vy_i/vx_i)*1.0)*sin(sigma_i-atan(vy_i/vx_i)*1.0)*1.0/pow(mu_t_p2+mu_t_p3*pow(sin(sigma_i-atan(vy_i/vx_i)*1.0),2.0),2.0)*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)*3.999999999999773E-1-(mu_n_p1*mu_n_p3*r*w3_i*cos(sigma_i+7.849999999999682E-1)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*3.999999999999773E-1)/vx_i-(mu_n_p1*mu_n_p3*r*w0_i*cos(sigma_i-7.849999999999682E-1)*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*3.999999999999773E-1)/vx_i)/Iw;
  DfDx(3,0) = r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))+mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w0_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0;
  DfDx(3,1) = r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)));
  DfDx(3,2) = r*cos(sigma_i-7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*1.999999999999886E-1;
  DfDx(3,3) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0)/It-(mu_n_p1*mu_n_p3*(r*r)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i;
  DfDx(3,10) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i-7.849999999999682E-1)*1.999999999999886E-1-vy_i*cos(sigma_i)*1.0+vx_i*sin(sigma_i))*-1.0+(mu_n_p1*mu_n_p3*(r*r)*w0_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w0_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w0_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i-7.849999999999682E-1)*1.999999999999886E-1+vx_i*cos(sigma_i)+vy_i*sin(sigma_i)-(r*w0_i*1.0)/It)*2.0)/vx_i;
  DfDx(4,0) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))+mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w1_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.0;
  DfDx(4,2) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*1.414776538334479E-1;
  DfDx(4,4) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0)))*-1.0)/It-(mu_n_p1*mu_n_p3*(r*r)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w1_i)/vx_i-1.0,2.0),2.0)*((r*w1_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1+vx_i-(r*w1_i*1.0)/It)*2.0)/vx_i;
  DfDx(5,0) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))-mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w2_i*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.0;
  DfDx(5,2) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*-1.414776538334479E-1;
  DfDx(5,5) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0)))*-1.0)/It+(mu_n_p1*mu_n_p3*(r*r)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w2_i)/vx_i-1.0,2.0),2.0)*((r*w2_i)/vx_i-1.0)*(vw_i*1.414776538334479E-1-vx_i*1.0+(r*w2_i)/It)*2.0)/vx_i;
  DfDx(6,0) = r*cos(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))-mu_n_p1*mu_n_p3*(r*r)*1.0/(vx_i*vx_i)*w3_i*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0;
  DfDx(6,1) = r*sin(sigma_i)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)));
  DfDx(6,2) = r*cos(sigma_i+7.849999999999682E-1)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.999999999999886E-1;
  DfDx(6,6) = ((r*r)*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*-1.0)/It+(mu_n_p1*mu_n_p3*(r*r)*cos(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i;
  DfDx(6,10) = r*(mu_n_p0+mu_n_p1/(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0)))*(vw_i*sin(sigma_i+7.849999999999682E-1)*1.999999999999886E-1+vy_i*cos(sigma_i)-vx_i*sin(sigma_i)*1.0)-(mu_n_p1*mu_n_p3*(r*r)*w3_i*sin(sigma_i)*1.0/pow(mu_n_p2+mu_n_p3*pow((r*w3_i*cos(sigma_i))/vx_i-1.0,2.0),2.0)*((r*w3_i*cos(sigma_i))/vx_i-1.0)*(vw_i*cos(sigma_i+7.849999999999682E-1)*1.999999999999886E-1-vx_i*cos(sigma_i)*1.0-vy_i*sin(sigma_i)*1.0+(r*w3_i)/It)*2.0)/vx_i;
  DfDx(7,0) = vx_i*cos(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)+(1.0/(vx_i*vx_i)*vy_i*sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i))/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0);
  DfDx(7,1) = vy_i*cos(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)-(sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*1.0)/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0));
  DfDx(7,9) = sin(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*-1.0;
  DfDx(8,0) = vx_i*sin(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)-(1.0/(vx_i*vx_i)*vy_i*cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i)*1.0)/(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0);
  DfDx(8,1) = vy_i*sin(chi_i+atan(vy_i/vx_i))*1.0/sqrt(vx_i*vx_i+vy_i*vy_i)+(cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i))/(vx_i*(1.0/(vx_i*vx_i)*(vy_i*vy_i)+1.0));
  DfDx(8,9) = cos(chi_i+atan(vy_i/vx_i))*sqrt(vx_i*vx_i+vy_i*vy_i);
  DfDx(9,2) = 1.0;
  DfDx(10,10) = -1.0/str_del;

  Eigen::MatrixXd DfDu(11,2);
  DfDu = Eigen::MatrixXd::Zero(11,2);
  DfDu(3,1) = k_mot;
  DfDu(4,1) = k_mot;
  DfDu(5,1) = k_mot;
  DfDu(6,1) = k_mot;
  DfDu(10,0) = 1.0/str_del;

  Eigen::MatrixXd DfDx1(11,11);
  DfDx1 = DfDx;

  if(DfDu.norm()+DfDp.norm()+x_d.norm() == 10){
    std::cout << "blah"  << std::endl;
  }

}
  double tdiff = spGeneralTools::Tock_us(t0);
  std::cout << "calc time: " << tdiff << " us" << std::endl;
  std::cout << "sum " << sum << std::endl;

  return 0;
}


/*
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
    double Iy = pp_[1];
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

    double str_in = 0.1;
    double Curr_in = 0.1;

    double r = 0.1;
    double theta = 0.785;
    double d = 0.2;
    double m=12;

    double w0 = w0_i;
    double w1 = w1_i;
    double w2 = w2_i;
    double w3 = w3_i;
    double sigma = sigma_i;

    double k0 = (r*w0 / vx_i*cos(sigma))-1;
    double k1 = (r*w1 / vx_i)-1;
    double k2 = (r*w2 / vx_i)-1;
    double k3 = (r*w3 / vx_i*cos(sigma))-1;

    double beta = atan2(vy_i,vx_i);
    double beta0 = beta - sigma;
    double beta1 = beta;
    double beta2 = beta;
    double beta3 = beta - sigma;


    str_in = u_t[0];
    Curr_in = u_t[1];

    r = 0.1;
    theta = 0.785;
    d = 0.2;
    m=12;

    double px = vx_i*Ix;
    double py = vy_i*Iy;
    double pw = vw_i*Iw;
    w0 = w0_i;
    w1 = w1_i;
    w2 = w2_i;
    w3 = w3_i;
    double chi = chi_i;
    sigma = sigma_i;

double It0 = It;
double It1 = It;
double It2 = It;
double It3 = It;

double tau = k_mot * Curr_in;

double c00 = sin(sigma);
double c01 = 0;
double c02 = 0;
double c03 = sin(sigma);
double c04 = cos(sigma);
double c05 = 0;
double c06 = 0;
double c07 = cos(sigma);
double c10 = cos(sigma);
double c11 = 0;
double c12 = 0;
double c13 = cos(sigma);
double c14 = -sin(sigma);
double c15 = 0;
double c16 = 0;
double c17 = -sin(sigma);
double c20 = d*cos(-sigma+theta);
double c21 = d*cos(theta);
double c22 = -d*cos(theta);
double c23 = -d*cos(theta+sigma);
double c24 = d*sin(theta-sigma);
double c25 = -d*sin(theta);
double c26 = -d*sin(theta);
double c27 = d*sin(sigma+theta);

double mu_n0 = mu_n_p0 + (mu_n_p1/(mu_n_p2+mu_n_p3*(k0^2)));
double mu_n1 = mu_n_p0 + (mu_n_p1/(mu_n_p2+mu_n_p3*(k1^2)));
double mu_n2 = mu_n_p0 + (mu_n_p1/(mu_n_p2+mu_n_p3*(k2^2)));
double mu_n3 = mu_n_p0 + (mu_n_p1/(mu_n_p2+mu_n_p3*(k3^2)));

double mu_t0 = mu_t_p0 + (mu_t_p1/(mu_t_p2+mu_t_p3*(sin(beta0)^2)));
double mu_t1 = mu_t_p0 + (mu_t_p1/(mu_t_p2+mu_t_p3*(sin(beta1)^2)));
double mu_t2 = mu_t_p0 + (mu_t_p1/(mu_t_p2+mu_t_p3*(sin(beta2)^2)));
double mu_t3 = mu_t_p0 + (mu_t_p1/(mu_t_p2+mu_t_p3*(sin(beta3)^2)));


double w0_d = tau - r*mu_n0 * (r*w0/It0 - c00*py/Iy - c10*px/Ix - c20*pw/Iw);
double w1_d = tau - r*mu_n1 * (r*w1/It1 - px/Ix - c21*pw/Iw);
double w2_d = tau - r*mu_n2 * (r*w2/It2 - px/Ix - c22*pw/Iw);
double w3_d = tau - r*mu_n3 * (r*w3/It3 - c03*py/Iy - c13*px/Ix - c23*pw/Iw);
double px_d = mu_n0 * c10 * (r*w0/It0 - c00*py/Iy - c10*px/Ix - c20*pw/Iw)
    + mu_n2 * (r*w2/It2 - px/Ix - c22*pw/Iw)
    + mu_n1 * (r*w1/It1 - px/Ix - c21*pw/Iw)
    - mu_t3 * c17 * (c27*pw/Iw + c17*px/Ix + c07*py/Iy)
    - mu_t0 * c14 * (c24*pw/Iw + c14*px/Ix + c04*py/Iy)
    + mu_n3 * c13 * (r*w3/It3 - c03*py/Iy - c13*px/Ix - c23*pw/Iw)
    + m * pw * py / (Iw * Iy);
double py_d = - mu_t0 * c04 * py * (c24*pw/Iw + c14*px/Ix + c04) / Iy
    - mu_t2 * (c26*pw/Iw + c14*px/Ix + c04)
    - mu_t1 * (c25*pw/Iw + py/Iy)
    + mu_n3 * c03 * (pw/Iw) * (r*w3/It3 - c03*py/Iy - c13*px/Ix - c23)
    + mu_n0 * c00 * (pw/Iw) * (r*w0/It0 - c00*py/Iy - c10*px/Ix - c20)
    - mu_t3 * c07 *  (c27*pw/Iw + c17*px/Ix + c07*py/Iy)
    - m * pw * px / (Ix * Ix);

double pw_d = - mu_t3 * c27 * (c27*pw/Iw + c17*px/Ix + c07*py/Iy)
    + mu_n0 * c20 * (r*w0/It0 - (c00*py/Iy + c10*px/Ix + c20*pw/Iw))
    + mu_n1 * c21 * (r*w1/It1 - (px/Ix + c21 * pw/Iw))
    + mu_n2 * c22 * (r*w2/It2 - (px/Ix + c22 * pw/Iw))
    + mu_n3 * c23 * (r*w3/It3 - (c03*py/Iy + c13*px/Ix + c23*pw/Iw))
    - mu_t0 * c24 * (c24*pw/Iw + c14*px/Ix + c04*py/Iy)
    - mu_t1 * c25 * (c25*pw/Iw + py/Iy)
    - mu_t2 * c26 * (c26*pw/Iw + py/Iy);

double V = sqrt(vx_i^2+vy_i^2);

// x_dot in inertial frame
double posex_d = V*cos(chi+beta);

// y_dot in inertial frame
double posey_d = V*sin(chi+beta);

// chi_dot = omega rotational speed
double chi_d = pw/Iw;

// first order delay function on steering
double sigma_d = (str_in-sigma)/str_del;

xd[0] = px_d/Ix;
xd[1] = py_d/Iy;
xd[2] = pw_d/Iw;
xd[3] = w0_d;
xd[4] = w1_d;
xd[5] = w2_d;
xd[6] = w3_d;
xd[7] = posex_d;
xd[8] = posey_d;
xd[9] = chi_d;
xd[10] = sigma_d;
*/
