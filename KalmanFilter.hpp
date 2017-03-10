// Portable EKF implementation;
// You will also need to implement a model() method for your application: basically obatain the Jacobian for the system, and model prediction.

#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp

#include <iostream>
#include "EKFStructure.hpp"


class KalmanFilter {
    
private:
  ekf_t ekf;
  int n;
  int m;
 
  void init(int n, int m);

public:
  // Initializes a TinyEKF object.
  KalmanFilter(int, int);
  ~KalmanFilter();

  double* getX() const;

  // Initialize the covariance matrix.
  void ekf_init(double* P_,double* Q_, double* R_, double* x_);

  //Setup the Jacobian of the sytem
  void model_update(double* fx_, double* hx_, double* F_, double* H_);
  
  //Performs one step of the prediction and update.
  int ekf_step(double* z);
 };

#endif
