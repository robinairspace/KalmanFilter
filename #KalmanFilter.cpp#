#include <iostream>
#include "KalmanFilter.hpp"
#include "MatOp.hpp"

KalmanFilter::KalmanFilter(int Nsta_, int Mobs_): n(Nsta_), m(Mobs_) 
{
  init( n, m); 
}

void KalmanFilter:: init( int n, int m)
{
    ekf.x = new double[n];
    ekf.P = new double[n*n];
    ekf.Q = new double[n*n];
    ekf.R = new double[m*m];
    ekf.G = new double[n*m];
    ekf.F= new double[n*n];
    ekf.H = new double[m*n];
    ekf.Ht = new double[n*m];
    ekf.Ft = new double[n*n];
    ekf.Pp = new double[n*n];
    ekf.fx = new double[n];
    ekf.hx = new double[m];
    ekf.tmp0 = new double[n*n];
    ekf.tmp1 = new double[n*m];
    ekf.tmp2 = new double[m*n];
    ekf.tmp3 = new double[m*m];
    ekf.tmp4 = new double[m*m];
    ekf.tmp5 = new double[m];

    // zero-out matrices 
    zeros(ekf.P, n, n);
    zeros(ekf.Q, n, n);
    zeros(ekf.R, m, m);
    zeros(ekf.G, n, m);
    zeros(ekf.F, n, n);
    zeros(ekf.H, m, n);
}

KalmanFilter::~KalmanFilter(){
  if (ekf.P){
    delete []ekf.x;
    delete []ekf.P;
    delete []ekf.Q; 
    delete []ekf.R;
    delete []ekf.G;
    delete []ekf.F;
    delete []ekf.H;
    delete []ekf.Ht;
    delete []ekf.Ft;
    delete []ekf.Pp;
    delete []ekf.fx;
    delete []ekf.hx;
    delete []ekf.tmp0;
    delete []ekf.tmp1;
    delete []ekf.tmp2;
    delete []ekf.tmp3;
    delete []ekf.tmp4;
    delete []ekf.tmp5;
  }
}
    
double* KalmanFilter::getX() const {
  return ekf.x;
}

void KalmanFilter::ekf_init(double* P_,double* Q_, double* R_, double* x_ )
{
  int i = 0;
    // Set Q, see [1]
  for (i= 0; i<n*n; ++i){
     ekf.P[i] = P_[i];
     ekf.Q[i] = Q_[i];
  }

    // initial covariances of state noise, measurement noise
  for (i= 0; i< m*m; ++i)
     ekf.R[i] = R_[i];

    // Set initial state
  for (i=0; i< n; ++i)
     ekf.x[i] = x_[i];
}

void KalmanFilter::model_update(double* fx_, double* hx_, double* F_, double* H_)
{
  int i = 0;
  for (i = 0; i<n; ++i)
    ekf.fx[i] = fx_[i];
  for (i = 0; i<m; ++i)
    ekf.hx[i] = hx_[i];

  for (i = 0; i<n*n; ++i)
    ekf.F[i] = F_[i];
  for (i = 0; i<m*n; ++i)
    ekf.H[i] = H_[i];
}

int KalmanFilter:: ekf_step(double* z)
{        
 
    // P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}
    mulmat(ekf.F, ekf.P, ekf.tmp0, n, n, n);
    transpose(ekf.F, ekf.Ft, n, n);
    mulmat(ekf.tmp0, ekf.Ft, ekf.Pp, n, n, n);
    accum(ekf.Pp, ekf.Q, n, n);

    // G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} 
    transpose(ekf.H, ekf.Ht, m, n);
    mulmat(ekf.Pp, ekf.Ht, ekf.tmp1, n, n, m);
    mulmat(ekf.H, ekf.Pp, ekf.tmp2, m, n, n);
    mulmat(ekf.tmp2, ekf.Ht, ekf.tmp3, m, n, m);
    accum(ekf.tmp3, ekf.R, m, m);
    if (cholsl(ekf.tmp3, ekf.tmp4, ekf.tmp5, m)) return 1;
    mulmat(ekf.tmp1, ekf.tmp4, ekf.G, n, m, m);

    // \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k)) 
    sub(z, ekf.hx, ekf.tmp5, m);
    mulvec(ekf.G, ekf.tmp5, ekf.tmp2, n, m);
    add(ekf.fx, ekf.tmp2, ekf.x, n);

    // P_k = (I - G_k H_k) P_k 
    mulmat(ekf.G, ekf.H, ekf.tmp0, n, m, n);
    negate(ekf.tmp0, n, n);
    mat_addeye(ekf.tmp0, n);
    mulmat(ekf.tmp0, ekf.Pp, ekf.P, n, n, n);

    // success 
    return 0;
}

