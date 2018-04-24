  //double ZeroJunctionp5v = (((w_0 / I_{t0}) + (w_3 / I_{t3})) + w_2 / I_{t2}) + w_1 / I_{t1};

  \dot{w_0} = \tau - r * \mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20});
  \dot{w_1} = \tau - r * \mu_{n1} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21});
  \dot{w_2} = \tau - r * \mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22});
  \dot{w_3} = \tau - r * \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23});
  \dot{p_\omega} = ((((((((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{27} + (\mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{20})) + (\mu_{n1} * c_{21} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21}))) + (\mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22}) * c_{22})) + (\mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{23})) - \mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{24}) - \mu_{t1} * ((p_\omega / I_\omega) * c_{25} + p_y / I_y) * c_{25}) - \mu_{t2} * ((p_\omega / I_\omega) * c_{26} + p_y / I_y) * c_{26};
  \dot{p_x} = ((((((\mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{10}) + \mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22})) + \mu_{n1} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21})) + ((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{17} + (-\mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{14}))) + m * (p_\omega / I_\omega) * (p_y / I_y)) + \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{13};
  \dot{p_y} = ((((-\mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{04} - \mu_{t2} * ((p_\omega / I_\omega) * c_{26} + p_y / I_y)) - \mu_{t1} * ((p_\omega / I_\omega) * c_{25} + p_y / I_y)) + \mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{03} + \mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{00}) + ((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{07}) - m * (p_\omega / I_\omega) * (p_x / I_x);

   \dot{p_\omega} = ((((((((p_\omega / I_\omega) * c_{27} + (p_x / I_x) * c_{17} + (p_y / I_y) * c_{07}) * -\mu_{t3} * c_{27} + (\mu_{n0} * (r * (w_0 / I_{t0}) - (p_y / I_y) * c_{00} - (p_x / I_x) * c_{10} - (p_\omega / I_\omega) * c_{20}) * c_{20})) + (\mu_{n1} * c_{21} * (r * (w_1 / I_{t1}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{21}))) + (\mu_{n2} * (r * (w_2 / I_{t2}) - (p_x / I_x) - (p_\omega / I_\omega) * c_{22}) * c_{22})) + (\mu_{n3} * (r * (w_3 / I_{t3}) - (p_y / I_y) * c_{03} - (p_x / I_x) * c_{13} - (p_\omega / I_\omega) * c_{23}) * c_{23})) - \mu_{t0} * ((p_\omega / I_\omega) * c_{24} + (p_x / I_x) * c_{14} + (p_y / I_y) * c_{04}) * c_{24}) - \mu_{t1} * ((p_\omega / I_\omega) * c_{25} + p_y / I_y) * c_{25}) - \mu_{t2} * ((p_\omega / I_\omega) * c_{26} + p_y / I_y) * c_{26};

    double p_omega_d = ((((((((p_omega / I_omega) * c27 + (p_x / I_x) * c17 + (p_y / I_y) * c07) * -mu_t3 * c27 + (mu_n0 * (r * (w0 / I_t0) - (p_y / I_y) * c00 - (p_x / I_x) * c10 - (p_omega / I_omega) * c20) * c20)) + (mu_n1 * c21 * (r * (w1 / I_t1) - (p_x / I_x) - (p_omega / I_omega) * c21))) + (mu_n2 * (r * (w2 / I_t2) - (p_x / I_x) - (p_omega / I_omega) * c22) * c22)) + (mu_n3 * (r * (w3 / I_t3) - (p_y / I_y) * c03 - (p_x / I_x) * c13 - (p_omega / I_omega) * c23) * c23)) - mu_t0 * ((p_omega / I_omega) * c24 + (p_x / I_x) * c14 + (p_y / I_y) * c04) * c24) - mu_t1 * ((p_omega / I_omega) * c25 + p_y / I_y) * c25) - mu_t2 * ((p_omega / I_omega) * c26 + p_y / I_y) * c26;
    double Jzpe = ((((((((Jzstate / Jzi) * TF_c27r + (Ixstate / Ixi) * TF_c17r + (Iystate / Iyi) * TF_c07r) * -TireLat3Rr * TF_c27r + (TireLon0Rr * (TireLon0TFr * (TireLon0Istate / TireLon0Ii) - (Iystate / Iyi) * TF_c00r - (Ixstate / Ixi) * TF_c10r - (Jzstate / Jzi) * TF_c20r) * TF_c20r)) + (TireLon1Rr * TF_c21r * (TireLon1TFr * (TireLon1Istate / TireLon1Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c21r))) + (TireLon2Rr * (TireLon2TFr * (TireLon2Istate / TireLon2Ii) - (Ixstate / Ixi) - (Jzstate / Jzi) * TF_c22r) * TF_c22r)) + (TireLon3Rr * (TireLon3TFr * (TireLon3Istate / TireLon3Ii) - (Iystate / Iyi) * TF_c03r - (Ixstate / Ixi) * TF_c13r - (Jzstate / Jzi) * TF_c23r) * TF_c23r)) - TireLat0Rr * ((Jzstate / Jzi) * TF_c24r + (Ixstate / Ixi) * TF_c14r + (Iystate / Iyi) * TF_c04r) * TF_c24r) - TireLat1Rr * ((Jzstate / Jzi) * TF_c25r + Iystate / Iyi) * TF_c25r) - TireLat2Rr * ((Jzstate / Jzi) * TF_c26r + Iystate / Iyi) * TF_c26r;

   >> der_w0 = diff(w0_d,sigma)
   der_w0 =
   -mu_n0*r*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega)

   >> der_w1 = diff(w1_d,sigma)
   der_w1 =
   0

   >> der_w2 = diff(w2_d,sigma)
   der_w2 =
   0

   >> der_w3 = diff(w3_d,sigma)
   der_w3 =
   mu_n3*r*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x)

   >> der_p_x = diff(p_x_d,sigma)
   der_p_x =
   mu_n0*sin(sigma)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega - (r*w0)/I_t0) + mu_n0*cos(sigma)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) - mu_t0*sin(sigma)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega) - mu_t0*cos(sigma)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) - mu_n3*sin(sigma)*((p_omega*cos(sigma + theta))/I_omega - (p_x*cos(sigma))/I_x - (p_y*sin(sigma))/I_y + (r*w3)/I_t3) - mu_n3*cos(sigma)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x) + mu_t3*cos(sigma)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x) - mu_t3*sin(sigma)*((p_x*cos(sigma))/I_x - (p_omega*cos(sigma + theta))/I_omega + (p_y*sin(sigma))/I_y)

   >> der_p_y = diff(p_y_d,sigma)
   der_p_y =
   mu_t0*cos(sigma)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega) - mu_n0*cos(sigma)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega - (r*w0)/I_t0) + mu_n0*sin(sigma)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) - mu_t0*sin(sigma)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) + mu_n3*cos(sigma)*((p_omega*cos(sigma + theta))/I_omega - (p_x*cos(sigma))/I_x - (p_y*sin(sigma))/I_y + (r*w3)/I_t3) + mu_t3*cos(sigma)*((p_x*cos(sigma))/I_x - (p_omega*cos(sigma + theta))/I_omega + (p_y*sin(sigma))/I_y) - mu_n3*sin(sigma)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x) + mu_t3*sin(sigma)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x)

   >> der_p_omega = diff(p_omega_d,sigma)
   der_p_omega =
   mu_n0*sin(sigma - theta)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega - (r*w0)/I_t0) + mu_n0*cos(sigma - theta)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) - mu_t0*sin(sigma - theta)*((p_x*cos(sigma))/I_x + (p_y*sin(sigma))/I_y + (p_omega*cos(sigma - theta))/I_omega) - mu_t0*cos(sigma - theta)*((p_x*sin(sigma))/I_x - (p_y*cos(sigma))/I_y + (p_omega*sin(sigma - theta))/I_omega) + mu_n3*sin(sigma + theta)*((p_omega*cos(sigma + theta))/I_omega - (p_x*cos(sigma))/I_x - (p_y*sin(sigma))/I_y + (r*w3)/I_t3) + mu_n3*cos(sigma + theta)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x) - mu_t3*cos(sigma + theta)*((p_omega*sin(sigma + theta))/I_omega + (p_y*cos(sigma))/I_y - (p_x*sin(sigma))/I_x) + mu_t3*sin(sigma + theta)*((p_x*cos(sigma))/I_x - (p_omega*cos(sigma + theta))/I_omega + (p_y*sin(sigma))/I_y)

   >> der2_w0 = diff(w0_d,tau)
   der2_w0 =
   1

   >> der2_w1 = diff(w1_d,tau)
   der2_w1 =
   1

   >> der2_w2 = diff(w2_d,tau)
   der2_w2 =
   1

   >> der2_w3 = diff(w3_d,tau)
   der2_w3 =
   1

   >> der2_p_x = diff(p_x_d,tau)
   der2_p_x =
   0

   >> der2_p_y = diff(p_y_d,tau)
   der2_p_y =
   0
   >> der2_p_omega = diff(p_omega_d,tau)
   der2_p_omega =
   0
