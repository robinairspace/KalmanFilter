
#ifndef SysModel_hpp
#define SysModel_hpp

#include<iostream>

class SysModel{

public:
  void model(ekf_t * ekf, double SV[4][3]);

private:
  int nStatus;
  int nOut;
}
#endif
