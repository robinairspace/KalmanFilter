#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include "KalmanFilter.hpp"
#include "MatOp.hpp"

using namespace std;

// positioning interval
static const double T = 1;
const int Nsta = 8;
const int Mobs = 4;

static void blkfill(ekf_t * ekf, const double * a, int off)
{
    off *= 2;

    ekf->Q[off*Nsta+off]   = a[0]; 
    ekf->Q[off*Nsta+off+1] = a[1];
    ekf->Q[(off+1)*Nsta+off]   = a[2];
    ekf->Q[(off+1)*Nsta+off+1] = a[3];
}


static void init(ekf_t * ekf)
{
    // Set Q
    const double Sf    = 36;
    const double Sg    = 0.01;
    const double sigma = 5;         // state transition variance
    const double Qb[4] = {Sf*T+Sg*T*T*T/3, Sg*T*T/2, Sg*T*T/2, Sg*T};
    const double Qxyz[4] = {sigma*sigma*T*T*T/3, sigma*sigma*T*T/2, sigma*sigma*T*T/2, sigma*sigma*T};

    blkfill(ekf, Qxyz, 0);
    blkfill(ekf, Qxyz, 1);
    blkfill(ekf, Qxyz, 2);
    blkfill(ekf, Qb,   3);

    // initial covariances of state noise, measurement noise
    double P0 = 10;
    double R0 = 36;

    int i;

    for (i=0; i<8; ++i)
        ekf->P[i*Nsta+i] = P0;

    for (i=0; i<4; ++i)
        ekf->R[i*Mobs+i] = R0;

    // position
    ekf->x[0] = -2.168816181271560e+006;
    ekf->x[2] =  4.386648549091666e+006;
    ekf->x[4] =  4.077161596428751e+006;

    // velocity
    ekf->x[1] = 0;
    ekf->x[3] = 0;
    ekf->x[5] = 0;

    // clock bias
    ekf->x[6] = 3.575261153706439e+006;

    // clock drift
    ekf->x[7] = 4.549246345845814e+001;
}

static void model(ekf_t* ekf, double SV[4][3])
{ 

    int i, j;
    //Model prediction
    for (j=0; j<8; j+=2) {
        ekf->fx[j] = ekf->x[j] + T * ekf->x[j+1];
        ekf->fx[j+1] = ekf->x[j+1];
    }
    //Update state transition matrix (Jacobian)
    for (j=0; j<8; ++j)
        ekf->F[j*Nsta+j] = 1;
    for (j=0; j<4; ++j)
        ekf->F[2*j*Nsta+2*j+1] = T;

    double dx[4][3];
    //Update output estimation
    for (i=0; i<4; ++i) {
        ekf->hx[i] = 0;
        for (j=0; j<3; ++j) {
            double d = ekf->fx[j*2] - SV[i][j];
            dx[i][j] = d;
            ekf->hx[i] += d*d;
        }
        ekf->hx[i] = pow(ekf->hx[i], 0.5) + ekf->fx[6];
    }
    //Update output matrix(Jacobian)
    for (i=0; i<4; ++i) {
        for (j=0; j<3; ++j) 
            ekf->H[i*Nsta+j*2]  = dx[i][j] / ekf->hx[i];
        ekf->H[i*Nsta+6] = 1;
    }   
}

static void readdata(ifstream& file, double SV_Pos[4][3], double SV_Rho[4])
{  
    string line;
    if ( !file.good() ) cout<<"file not good"<<endl;
    getline(file, line);

    stringstream iss(line);

    for (int col = 0; col < 16; ++col)
    {
        string val;
        getline(iss, val, ',');
        if ( !iss.good() ) 
            break;

        stringstream convertor(val);
        if (col<12) convertor >> SV_Pos[col/3][col%3];
	else convertor >> SV_Rho[col-12];
    }
}

int main(int argc, char ** argv)
{   
  //int Nsta = 8; int Mobs = 4;
    ekf_t ekf;
    allocate(ekf, Nsta, Mobs);
    init(&ekf);

    KalmanFilter EKF(Nsta, Mobs);
    // Do generic EKF initialization, Do local initialization
    EKF.ekf_init(ekf.P, ekf.Q, ekf.R, ekf.x);

    // Open input data file
    ifstream ifile("gps.csv");
    double SV_Pos[4][3];
    double SV_Rho[4];
    double Pos_KF[25][3];

    int j, k;
    string line;
    // Loop till no more data
    for (j=0; j<25; ++j) {
       if ( !ifile.good() ) break;
       getline(ifile, line);
       stringstream iss(line);
       for (int col = 0; col < 16; ++col)
       {
	 string val;
	 getline(iss, val, ',');
	 if ( !iss.good() ) 
            break;

	 stringstream convertor(val);
	 if (col<12) convertor >> SV_Pos[col/3][col%3];
	 else convertor >> SV_Rho[col-12];
       }

 	model(&ekf, SV_Pos);
        EKF.model_update(ekf.fx, ekf.hx, ekf.F, ekf.H);
        EKF.ekf_step(SV_Rho);

        // grab positions, ignoring velocities
	ekf.x = EKF.getX();
        for (k=0; k<3; ++k)
	  Pos_KF[j][k] = ekf.x[2*k];
    }

    ifile.close();

    // Compute means of filtered positions
    double mean_Pos_KF[3] = {0, 0, 0};
    for (j=0; j<25; ++j) 
        for (k=0; k<3; ++k)
            mean_Pos_KF[k] += Pos_KF[j][k];
    for (k=0; k<3; ++k)
        mean_Pos_KF[k] /= 25;

    // Open output CSV file and write header
    const char  * OUTFILE = "ekf.csv";
    ofstream ofile;
    ofile.open(OUTFILE);

    // Dump filtered positions minus their means
    for (j=0; j<25; ++j) {
      ofile<< Pos_KF[j][0]-mean_Pos_KF[0]<<" "<< Pos_KF[j][1]-mean_Pos_KF[1]<<" "<< Pos_KF[j][2]-mean_Pos_KF[2]<<endl;
      cout<< Pos_KF[j][0]<<","<< Pos_KF[j][1]<<","<< Pos_KF[j][2]<<endl;
    }
    ofile.close();
    empty(ekf);
    return 0;
}


