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

// Types
enum NavState {Calibration, INS, GPS};

struct SensorErrorType {
    double sum;
    double mean;
    double max;
    double min;
    double std;
    double sumSqr;
    double variance;
    
    SensorErrorType() : sum(0), mean(0), max(-999), min(999), std(0), sumSqr(0), variance(0) {}
};

struct NavType {
    double position[3]; //Latitude, longitude, Altitude
    double velNED[3];
    bool   initNED;
    double eulerAngles[3];
    double q_B_NED[4]; // Quaternion of Body relative to NED
    double accBias[3];
    double gyroBias[3];
    double gravity;
    double velBody[3];
    
    NavState state;
    double timestamp;
    
    NavType()
    {
        state = Calibration;
        timestamp = 0.0;
        gravity = 9.81;
        initNED = false;
        for (int i=0; i<3; i++)
        {
            position[i]    = 0.0;
            velNED[i]      = 0.0;
            velBody[i]     = 0.0;
            eulerAngles[i] = 0.0;
            accBias[i]     = 0.0;
            gyroBias[i]    = 0.0;
            q_B_NED[i]     = 0.0;
        }
        q_B_NED[3] = 0.0;
    }
};

// Setup
void FsNavigation_setupNavigation(double *initialPosition, double initialHeading);

// Perform
void FsNavigation_performNavigation( double &navDt );

void updateGravity();
void performARHS(double &navDt);
void gyroUpdate(double &navDt);
void compFilter();
void performINS( double &navDt );
void FsNavigation_performGPSPVTUpdate(double* gps_LLA, double* gps_velNED, double gps_heading, double gps_timestamp);

// Calibration
void FsNavigation_calibrateIMU();

// Getters
NavType* FsNavigation_getNavData(bool useTruth = false);
NavType* FsNavigation_getNavError();
NavState FsNavigation_getNavState();

// Setters
void FsNavigation_setIMUdata(IMUtype* pIMUdataIn);
void FsNavigation_initNED(double* LLA, double* velNED, double heading, bool bypassInit = false);
#ifdef SIMULATION
    void FsNavigation_setSimulationModels(ModelMap* pMap);
#endif

// Support
#ifdef SIMULATION
    void updateTruth();
#endif

inline void FsNavigation_bodyToNED(double* vNED, double* vB);
inline void FsNavigation_NEDToBody(double* vB, double* vNED);
void FsNavigation_bodyToLL(double* vLL, double* vBody);
inline void quaternionProduct(double *product, double *q1, double *q2);
inline void updateEulerAngles();

#endif /* fs_navigation_hpp */
