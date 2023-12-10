//
//  fs_navigation.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/20/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_navigation_hpp
#define fs_navigation_hpp

#include "fs_common.hpp"
#include "fs_barometer.hpp"
#include "fs_gps.hpp"

#define FILTERTEST

// Types
enum NavState {Nav_Startup, Calibration, INS, AccelUpdate, GPSUpdate, GPSUpdate2D, BaroUpdate, GroundAlign, NNAVSTATES};
enum StateType {ATT_X, ATT_Y, ATT_Z, VN, VE, VD, N, E, ALT, GBIAS_X, GBIAS_Y, GBIAS_Z, ABIAS_X, ABIAS_Y, ABIAS_Z, GRAVITY, NSTATES};
enum VECTORTYPE {X, Y, Z};
enum AccelUpdateType {ACCEL_ROLL, ACCEL_PITCH, NACCELSTATES};
enum GroundMeasurementType {GROUND_N, GROUND_E, GROUND_ALT, GROUND_VN, GROUND_VE, GROUND_VD, GROUND_YAW, NGROUNDSTATES};
enum GPSMeasurementType {GPS_N, GPS_E, GPS_ALT, GPS_VN, GPS_VE, GPS_VD, NGPSSTATES};
enum GPS2DMeasurementType {GPS2D_N, GPS2D_E, GPS2D_VN, GPS2D_VE, N2DGPSSTATES};
enum BAROMeasurementType {BARO_ALT, NBAROSTATES};

struct StateInputType {
    double dTheta[3];
    double dVelocity[3];
    
    StateInputType()
    {
        for (int i=0; i<3; i++)
        {
            dTheta[i] = 0.0;
            dVelocity[i] = 0.0;
        }
    }
};

struct NavType {
    double position[3]; // North, East, Altitude
    double velNED[3];
    bool   initNED;
    bool   allowLoadIMUCal;
    double eulerAngles[3];
    double q_B_NED[4]; // Quaternion of Body relative to NED
    double accBias[3];
    double gyroBias[3];
    double gravityBias;
    double altitude_msl;
    double geoidCorrection;
    double gravity;
    double velBody[3];
    double accelBody[3];
    double gravityBody[3];
    double bodyRates[3];
    StateInputType stateInputs;
    
    double accel_pitch;
    double accel_roll;
    double accel_mag;
    double rates_mag;
    
    NavState     state;
    double       timestamp;
    double       imuTimestamp;
    unsigned int updateCount[NNAVSTATES];
    unsigned int skippedUpdateCount[NNAVSTATES];
    double       sensorTimestamp[NNAVSTATES];
    double       timestamp_diff[NNAVSTATES];
    
    NavType()
    {
        state = Nav_Startup;
        timestamp = 0.0;
        gravity = (double) (Gravity);
        altitude_msl = 0.0;
        geoidCorrection = 0.0;
        initNED = false;
        allowLoadIMUCal = false;
        
        accel_pitch = 0.0;
        accel_roll  = 0.0;
        accel_mag   = 0.0;
        rates_mag   = 0.0;
        gravityBias = 0.0;
        for (int i=0; i<3; i++)
        {
            position[i]    = 0.0;
            velNED[i]      = 0.0;
            velBody[i]     = 0.0;
            eulerAngles[i] = 0.0;
            accBias[i]     = 0.0;
            gyroBias[i]    = 0.0;
            q_B_NED[i]     = 0.0;
            accelBody[i]   = 0.0;
        }
        q_B_NED[3] = 0.0;
        
        for (int i=0; i<NNAVSTATES; i++)
        {
            updateCount[i]     = 0;
            sensorTimestamp[i] = 0.0;
            timestamp_diff[i]  = 0.0;
        }
    }
};

// Setup
void FsNavigation_setupNavigation(double *initialPosition, double initialHeading, bool loadIMUCalibration);

// Perform
void FsNavigation_performNavigation( double &navDt );

void updateGravity();
void updateInputs(double &navDt);
void performARHS(double &navDt);
void pitchRollAccelerometer();
void gyroUpdate(double &navDt);
void performINS( double &navDt );
void propogateVariance( double &navDt );
void applyCorrections();

// Filter Updates
void filterUpdate(double* residual, double* R, double* H, double* K, int nMeas);
void FsNavigation_performAccelerometerUpdate(bool performUpdate);
void FsNavigation_performGPSUpdate(const GpsType* gpsData);
void FsNavigation_performBarometerUpdate(const BarometerType* baroData);
void FsNavigation_groundAlign();

// Calibration
void FsNavigation_calibrateIMU();

// Getters
const NavType* FsNavigation_getNavData(bool useTruth = false);
double   FsNavigation_getNavAlt();
const NavType* FsNavigation_getNavError();
NavState FsNavigation_getNavState();
const double*  FsNavigation_getCovariance();
const double*  FsNavigation_getProcessNoise();
const double*  FsNavigation_getStateError();
const double*  FsNavigation_getAccumStateError();
const double*  FsNavigation_getCovarianceCorrection();
const double*  FsNavigation_getBaroKalmanGain();
const double*  FsNavigation_getBaroResidual();
const double*  FsNavigation_getGPSMeasVariance();
const double*  FsNavigation_getGPSResidual();
const unsigned int* FsNavigation_getGPSFilterError();
const double*  FsNavigation_getAccelMeasVariance();
const double* FsNavigation_getGroundResidual();

const SensorErrorType* FsNavigation_getGyroStatistics();
const SensorErrorType* FsNavigation_getAccelStatistics();

// Setters
void FsNavigation_setIMUdata(const IMUtype* pIMUdataIn);
void FsNavigation_setNED(double* LLA, double* velNED, double heading, bool bypassInit = false);
void FsNavigation_setGroundFlags(bool onGround, bool movingDetection);
#ifdef SIMULATION
    void FsNavigation_setSimulationModels(ModelMap* pMap);
    void FsNavigation_setTruthTransitionMatrix();
#endif

// Support
#ifdef SIMULATION
   void updateTruth();
   void computeTruthErrors();
#endif

inline void FsNavigation_bodyToNED(double* vNED, const double* vB);
inline void FsNavigation_NEDToBody(double* vB, const double* vNED);
void FsNavigation_bodyToLL(double* vLL, const double* vBody);
inline void quaternionProduct(double *product, double *q1, double *q2);
inline void updateEulerAngles();
inline void updateQuaternions();

// Math
inline void unitVector(double* vector, int n);
void math_mgain(double *result, double *matrix, double gain, int nrow, int ncol);
void math_madd(double *result, double *A, double *B, int nrows, int ncols);
void math_msubtract(double *result, double *A, double *B, int nrows, int ncols);
void math_mmult(double *result, double *A, int nrows1, int ncols1, double *B, int nrows2, int ncols2);
void math_mmult(double *result, double *matrix, double *vector, int nrow, int ncol);
void math_mtran(double *matrix_t, double *matrix, int nrow_t, int ncol_t);
void math_minv(double *matrix_inv, double *matrix, int n);
void math_LUdecomp(double *x, double *A, double *b, int n);
inline void symmetric(double* matrix, int n, int m);

#endif /* fs_navigation_hpp */
