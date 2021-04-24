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
    
    void printMean()
    {
        display("mean: ");
        display(mean);
        display("\n");
    }
    
    void print()
    {
        display("sum: ");
        display(sum);
        
        display(", mean: ");
        display(mean);
        
        display(", max: ");
        display(max);
        
        display(", min: ");
        display(min);
        
        display(", std: ");
        display(std);
        
        display(", sumSqr: ");
        display(sumSqr);
        
        display(", variance: ");
        display(variance);
        display("\n");
    }
};

struct NavType {
    double position[3]; // Latitude, longitude, wgs84 altitde
    double mslAlt;
    double velNED[3];
    double velBody[3];
    double speed;
    
    double accelBody[3];
    double accelIMUBody[3];
    double gravityNED[3];
    double gravityBody[3];
    double gravity;
    
    double dVelocity[3];
    double dTheta[3];
    
    double eulerAngles[3];
    double eulerRates[3];
    double bodyRates[3];
   
    double q_B_NED[4]; // Quaternion of Body relative to NED

    double accBias[3];
    double gyroBias[3];
    double gyroBiasDrift[3];
    
    double dVelBias[3];
    double dThetaBias[3];
    
    bool useAcc;
    bool useMag;
    
    double imuDt;
    NavState state;
    double timestamp;
    
    NavType()
    {
        mslAlt  = 0.0;
        speed   = 0.0;
        gravity = 0.0;
        useAcc  = false;
        useMag  = false;
        imuDt   = 0.0;
        state   = Calibration;
        timestamp = 0.0;
        
        for (int i=0; i<3; i++)
        {
            position[i]    = 0.0;
            velNED[i]      = 0.0;
            velBody[i]     = 0.0;
            accelIMUBody[i] = 0.0;
            accelBody[i]   = 0.0;
            gravityNED[i]  = 0.0;
            gravityBody[i] = 0.0;
            dVelocity[i]   = 0.0;
            dTheta[i]      = 0.0;
            eulerAngles[i] = 0.0;
            eulerRates[i]  = 0.0;
            bodyRates[i]   = 0.0;
            accBias[i]     = 0.0;
            gyroBias[i]    = 0.0;
            gyroBiasDrift[i] = 0.0;
            dVelBias[i]    = 0.0;
            dThetaBias[i]  = 0.0;
            q_B_NED[i]     = 0.0;
        }
        q_B_NED[3] = 0.0;
    }
};

// Setup
void FsNavigation_setupNavigation(double *initialPosition);

// Perform
void FsNavigation_performNavigation();

void applyCalibration();
void performARHS();
void performINS();
void performEKF();

// Calibration
void FsNavigation_calibrateIMU();
void FsNavigation_calibrateMAG();

// Getters
NavType* FsNavigation_getNavData(bool useTruth = false);
NavType* FsNavigation_getNavError();

// Setters
void FsNavigation_setIMUdata(IMUtype* pIMUdataIn);
void FsNavigation_setSimulationModels(ModelMap* pMap);

// Support
void updateTruth();

void FsNavigation_bodyToNED(double* vNED, double* vB);
void FsNavigation_NEDToBody(double* vB, double* vNED);

void quaternionProduct(double *product, double *q1, double *q2);

#endif /* fs_navigation_hpp */
