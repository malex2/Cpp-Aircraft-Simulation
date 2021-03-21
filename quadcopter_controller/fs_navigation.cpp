//
//  fs_navigation.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/20/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_navigation.hpp"
#include "fs_imu.hpp"

// Data
NavType NavData;
IMUtype* nav_pIMUdata = 0;

double navDt = 1.0/50.0;

// Booleans
bool navSetup = false;
bool imuCalibrated = false;
bool magCalibrated = false;

// Accelerometer Acceptance Criteria
double maxFilterAcc;
double minFilterAcc;
double bodyRateThresh;

// Sensor Errors
SensorErrorType accelError[3];
SensorErrorType gyroError[3];
int calCounter;
int maxCalCounter;
double sensorToBody[3];

// Madgwick Filter Parameters
double beta;
double zeta;

void FsNavigation_setupNavigation(double *initialPosition)
{
    // Nav Data
    NavData.position[0] = initialPosition[0];
    NavData.position[1] = initialPosition[1];
    NavData.position[2] = initialPosition[2];
    NavData.mslAlt = initialPosition[2] + 90*0.3048;
    
    NavData.gravity = 9.81;
    
    NavData.q_B_NED[0] = 1.0;
    NavData.q_B_NED[1] = 0.0;
    NavData.q_B_NED[2] = 0.0;
    NavData.q_B_NED[3] = 0.0;
    
    NavData.state = Calibration;
    
    NavData.useAcc = false; // automatically updated
    NavData.useMag = false; // set true/false here
    
    // Variables
    maxFilterAcc    = 1.05; // g's
    minFilterAcc    = 0.95; // g's
    bodyRateThresh  = 1.0;  // deg/s
    magCalibrated = !NavData.useMag;
    
    beta = 0.0;
    zeta = 0.015;
    
    sensorToBody[0] = 1;
    sensorToBody[1] = -1;
    sensorToBody[2] = -1;
    
    // Calibration
    calCounter    = 0;
    maxCalCounter = 100;

    navSetup = true;
}

void FsNavigation_performNavigation()
{
    static double prevIMUtime = 0.0;
    NavData.imuDt = nav_pIMUdata->timestamp - prevIMUtime;
    prevIMUtime = nav_pIMUdata->timestamp;
    
    // Determine Nav State
    if (imuCalibrated && magCalibrated) { NavData.state = INS; }
    
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    applyCalibration();
    
    // Attitude and Heading Reference System
    performARHS();
    
    // Update full Navigation solution
    performINS();
}

void applyCalibration()
{
    for (int i=0; i<3; i++)
    {
        NavData.bodyRates[i] = sensorToBody[i]*nav_pIMUdata->gyro[i]*deg2rad          - NavData.gyroBias[i];
        NavData.accelBody[i] = sensorToBody[i]*nav_pIMUdata->accel[i]*NavData.gravity - NavData.accBias[i];
        
        NavData.dVelBias[i]   = NavData.gyroBias[i] * NavData.imuDt;
        NavData.dThetaBias[i] = NavData.accBias[i]  * NavData.imuDt;
        
        NavData.dVelocity[i] = sensorToBody[i]*nav_pIMUdata->dVelocity[i]*NavData.gravity - NavData.dVelBias[i];
        NavData.dTheta[i]    = sensorToBody[i]*nav_pIMUdata->dTheta[i]*deg2rad - NavData.dThetaBias[i];
    }
}

void performARHS()
{
    double bodyRateMag;
    double dThetaMag;
    double aMag;
    double qMag;
    double accelUnit[3];
    double dThetaUnit[3];
    int zdir = -1;
    double f[6];
    double Jtrans[4][6];
    
    double dqGyro[4];
    double qw[4];
    double qedot[4];
    double df[4];
    double dfMag;
    
    // Shorthand
    double* q = NavData.q_B_NED;
    
    // Magnitudes and Unit Vectors
    bodyRateMag = sqrt( NavData.bodyRates[0]*NavData.bodyRates[0] +
                        NavData.bodyRates[1]*NavData.bodyRates[1] +
                        NavData.bodyRates[2]*NavData.bodyRates[2] );
    aMag        = sqrt( NavData.accelBody[0]*NavData.accelBody[0] +
                        NavData.accelBody[1]*NavData.accelBody[1] +
                        NavData.accelBody[2]*NavData.accelBody[2] );
    
    for (int i=0; i<3; i++)
    {
        accelUnit[i]  = NavData.accelBody[i] / aMag;
    }
    
    // Assess Acceleration
    NavData.useAcc = (aMag < maxFilterAcc) && (aMag > minFilterAcc) && (bodyRateMag < bodyRateThresh);
    
    // Zero Cost Function
    for (int i=0; i<6; i++) { f[i] = 0.0; }
    
    // accelerometer part of cost function
    if (NavData.useAcc)
    {
        f[0] = 2*zdir*(q[1]*q[3] - q[0]*q[2])       - accelUnit[0];
        f[1] = 2*zdir*(q[0]*q[1] + q[2]*q[3])       - accelUnit[1];
        f[2] = 2*zdir*(0.5 - q[1]*q[1] - q[2]*q[2]) - accelUnit[2];
        
        // accelerometer part of cost function gradient
        //Jtrans[0][0] = -2*zdir*q[2]; Jtrans[0][1] = 2*zdir*q[3];  Jtrans[0][2] = -2*zdir*q[0]; Jtrans[0][3] = 2*zdir*q[1];
        //Jtrans[1][0] = 2*zdir*q[1];  Jtrans[1][1] = 2*zdir*q[0];  Jtrans[1][2] = 2*zdir*q[3];  Jtrans[1][3] = 2*zdir*q[2];
        //Jtrans[2][0] = 0.0;          Jtrans[2][1] = -4*zdir*q[1]; Jtrans[2][2] = -4*zdir*q[2]; Jtrans[2][3] = 0.0;
        
        Jtrans[0][0] = -2*zdir*q[2]; Jtrans[0][1] = 2*zdir*q[1]; Jtrans[0][2] = 0.0;
        Jtrans[1][0] =  2*zdir*q[3]; Jtrans[1][1] = 2*zdir*q[0]; Jtrans[1][2] = -4*zdir*q[1];
        Jtrans[2][0] = -2*zdir*q[0]; Jtrans[2][1] = 2*zdir*q[3]; Jtrans[2][2] = -4*zdir*q[2];
        Jtrans[3][0] =  2*zdir*q[1]; Jtrans[3][1] = 2*zdir*q[2]; Jtrans[3][2] = 0.0;
    }
    
    // magnetometer part of cost function
    if (NavData.useMag)
    {
        // TODO
    }
    
    // correction gradient
    df[0] = Jtrans[0][0]*f[0] + Jtrans[0][1]*f[1] + Jtrans[0][2]*f[2] + Jtrans[0][3]*f[3] + Jtrans[0][4]*f[4];
    df[1] = Jtrans[1][0]*f[0] + Jtrans[1][1]*f[1] + Jtrans[1][2]*f[2] + Jtrans[1][3]*f[3] + Jtrans[1][4]*f[4];
    df[2] = Jtrans[2][0]*f[0] + Jtrans[2][1]*f[1] + Jtrans[2][2]*f[2] + Jtrans[2][3]*f[3] + Jtrans[2][4]*f[4];
    df[3] = Jtrans[3][0]*f[0] + Jtrans[3][1]*f[1] + Jtrans[3][2]*f[2] + Jtrans[3][3]*f[3] + Jtrans[3][4]*f[4];
    dfMag = sqrt(df[0]*df[0] + df[1]*df[1] + df[2]*df[2] + df[3]*df[3]);
    
    // quaternion error rate
    for (int i = 0; i < 4; i++)
    {
        // do not divide by zero
        if (dfMag > 0.01) { qedot[i] = df[i]/dfMag; }
        else              { qedot[i] = 0.0; }
    }
    
    // Gyroscope drift compensation
    // Only apply when using magnetometer for bias to be observable
    if (NavData.useMag)
    {
        // Compute gyroscope bias drift
        double omegaError[4];
        quaternionProduct(omegaError, q, qedot);
        for (int i=0; i<4; i++) { omegaError[i] *= 2; }
        
        // Apply gyroscope bias drift
        for (int i=0; i<3; i++)
        {
            NavData.gyroBiasDrift[i] += zeta*omegaError[i+1]*navDt;
            NavData.bodyRates[i] -= NavData.gyroBiasDrift[i];
            NavData.dTheta[i]    -= NavData.gyroBiasDrift[i]*NavData.imuDt;
        }
    }
    
    // dTheta magnitude and unit vector
    dThetaMag = sqrt( NavData.dTheta[0]*NavData.dTheta[0] +
                      NavData.dTheta[1]*NavData.dTheta[1] +
                      NavData.dTheta[2]*NavData.dTheta[2] );
    
    for (int i=0; i<3; i++)
    { dThetaUnit[i] = NavData.dTheta[i] / dThetaMag; }

    // Quaternion from Gyroscope
    dqGyro[0] = cos(dThetaMag);
    dqGyro[1] = dThetaUnit[0]*sin(dThetaMag);
    dqGyro[2] = dThetaUnit[1]*sin(dThetaMag);
    dqGyro[3] = dThetaUnit[2]*sin(dThetaMag);
    
    quaternionProduct(qw, q, dqGyro);
    
    // Correct using accelerometer and magnetometer
    for (int i=0; i<4; i++)
    { NavData.q_B_NED[i] = qw[i] - beta*qedot[i]*navDt; }
    
    // gaureentee unit vector
    qMag = sqrt( NavData.q_B_NED[0]*NavData.q_B_NED[0] +
                 NavData.q_B_NED[1]*NavData.q_B_NED[1] +
                 NavData.q_B_NED[2]*NavData.q_B_NED[2] +
                 NavData.q_B_NED[3]*NavData.q_B_NED[3] );
    
    for (int i=0; i<4; i++)
    { NavData.q_B_NED[i] /= qMag; }
}

void performINS()
{
    // Shorthand
    double* q = NavData.q_B_NED;
    
    // Euler Angles
    NavData.eulerAngles[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    NavData.eulerAngles[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    NavData.eulerAngles[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
    
    // Euler Rates
    double cr = cos(NavData.eulerAngles[0]);     // cos(roll)
    double sr = sin(NavData.eulerAngles[0]);     // sin(roll)
    double tp = tan(NavData.eulerAngles[1]);     // tan(pitch)
    double secp = 1/cos(NavData.eulerAngles[1]);  // sec(pitch)
    
    NavData.eulerRates[0] = 1.0*NavData.bodyRates[0] + sr*tp  *NavData.bodyRates[1] + cr*tp  *NavData.bodyRates[2];
    NavData.eulerRates[1] =                            cr     *NavData.bodyRates[1] + -sr    *NavData.bodyRates[2];
    NavData.eulerRates[2] =                            sr*secp*NavData.bodyRates[1] + cr*secp*NavData.bodyRates[2];

    // Body Velocity
    for (int i=0; i<3; i++)
    NavData.velBody[i] += NavData.dVelocity[i];
    
    // NED Velocity
    FsNavigation_bodyToNED(NavData.velNED, NavData.velBody);
    
    // Altitude
    NavData.mslAlt -= NavData.velNED[2]*navDt;
    NavData.position[2] -= NavData.velNED[2]*navDt;
    
    // Latitude, Longitude
    NavData.position[0] += NavData.velNED[0]/(NavData.position[2] + RE) * navDt;
    NavData.position[1] += NavData.velNED[1]/(NavData.position[2] + RE) * navDt;
}

void FS_performGPSUpdate()
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    NavData.state = GPS;
}

void FsNavigation_calibrateIMU()
{
    if (imuCalibrated) { return; }
    double tempAcc;
    double tempGyro;
    double sum_std;
    
    // Get Sum
    for (int i=0; i<3; i++)
    {
        // Gyroscope
        tempGyro = sensorToBody[i] * nav_pIMUdata->gyro[i] * deg2rad;
        gyroError[i].sum    += tempGyro;
        gyroError[i].sumSqr += tempGyro * tempGyro;
        if (nav_pIMUdata->gyro[i] > gyroError[i].max) { gyroError[i].max =  tempGyro; }
        if (nav_pIMUdata->gyro[i] < gyroError[i].min) { gyroError[i].min =  tempGyro; }
        
        // Accelerometer
        if (i==2) { tempAcc = sensorToBody[i] * (nav_pIMUdata->accel[i] - 1.0) * NavData.gravity; }
        else { tempAcc = sensorToBody[i] * nav_pIMUdata->accel[i] * NavData.gravity; }
        
        accelError[i].sum    += tempAcc;
        accelError[i].sumSqr += tempAcc * tempAcc;
        if (tempAcc > accelError[i].max) { accelError[i].max =  tempAcc; }
        if (tempAcc < accelError[i].min) { accelError[i].min =  tempAcc; }
    }
    
    calCounter++;
    
    // Computer statistics
    if (calCounter >= maxCalCounter)
    {
        for (int i=0; i<3; i++)
        {
            gyroError[i].mean = gyroError[i].sum / maxCalCounter;
            sum_std = gyroError[i].sumSqr - maxCalCounter*gyroError[i].mean - 2*gyroError[i].mean*gyroError[i].sum;
            gyroError[i].variance = sum_std / maxCalCounter;
            gyroError[i].std = sqrt(gyroError[i].variance);
            
            accelError[i].mean = accelError[i].sum / maxCalCounter;
            sum_std = accelError[i].sumSqr - maxCalCounter*accelError[i].mean - 2*accelError[i].mean*accelError[i].sum;
            accelError[i].variance = sum_std / maxCalCounter;
            accelError[i].std = sqrt(accelError[i].variance);
        }
        
        // gyroscope measurement error
        beta = sqrt(3/4) * 3 * (gyroError[0].variance + gyroError[1].variance + gyroError[2].variance);
        imuCalibrated = true;
    }
}

void FsNavigation_calibrateMAG();

NavType* FsNavigation_getNavData() { return &NavData; }

void FsNavigation_setIMUdata(IMUtype* pIMUdataIn)
{
    nav_pIMUdata = pIMUdataIn;
}

void FsNavigation_bodyToNED(double* vNED, double* vB)
{
    // vNED = q_B_NED * vB * q_B_NED'
    double q1 = NavData.q_B_NED[0];
    double q2 = NavData.q_B_NED[1];
    double q3 = NavData.q_B_NED[2];
    double q4 = NavData.q_B_NED[3];
    
    vNED[0] = (2*q1*q1-1+2*q2*q2)*vB[0] + 2*(q2*q3 + q1*q4)  *vB[1] + 2*(q2*q4 - q1*q3)  *vB[2];
    vNED[1] = 2*(q2*q3 - q1*q4)  *vB[0] + (2*q1*q1-1+2*q3*q3)*vB[1] + 2*(q3*q4 + q1*q2)  *vB[2];
    vNED[2] = 2*(q2*q4 + q1*q3)  *vB[0] + 2*(q3*q4 - q1*q2)  *vB[1] + (2*q1*q1-1+2*q4*q4)*vB[2];

}

void FsNavigation_NEDToBody(double* vB, double* vNED)
{
    // vB = q_B_NED' * vNED * q_B_NED
    
    double q1 =  NavData.q_B_NED[0];
    double q2 = -NavData.q_B_NED[1];
    double q3 = -NavData.q_B_NED[2];
    double q4 = -NavData.q_B_NED[3];
    
    vNED[0] = (2*q1*q1-1+2*q2*q2)*vB[0] + 2*(q2*q3 + q1*q4)  *vB[1] + 2*(q2*q4 - q1*q3)  *vB[2];
    vNED[1] = 2*(q2*q3 - q1*q4)  *vB[0] + (2*q1*q1-1+2*q3*q3)*vB[1] + 2*(q3*q4 + q1*q2)  *vB[2];
    vNED[2] = 2*(q2*q4 + q1*q3)  *vB[0] + 2*(q3*q4 - q1*q2)  *vB[1] + (2*q1*q1-1+2*q4*q4)*vB[2];
    
}

void quaternionProduct(double *product, double *q1, double *q2)
{
    product[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    product[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    product[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    product[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}
