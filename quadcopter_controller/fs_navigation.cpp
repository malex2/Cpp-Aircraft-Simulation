//
//  fs_navigation.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/20/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_navigation.hpp"
#include "fs_imu.hpp"
#ifdef SIMULATION
    #include "dynamics_model.hpp"
    #include "atmosphere_model.hpp"
#endif

// Data
NavType NavData;
IMUtype* nav_pIMUdata = 0;
#ifdef SIMULATION
    NavType truthNavData;
    NavType NavError;
#endif

// Booleans
bool navSetup = false;
bool imuCalibrated = false;
bool customPrint = false;

// Sensor Errors
SensorErrorType* accelError[3];
SensorErrorType* gyroError[3];
int calCounter;
int maxCalCounter;

// Complemintary Filter Parameters
double gyroVar;
double accelVar;

// Simulation Models
#ifdef SIMULATION
    class DynamicsModel*   pDyn = 0;
    class AtmosphereModel* pAtmo = 0;
#endif

void FsNavigation_setupNavigation(double *initialPosition, double initialHeading)
{
    double initPosition[3];
    double initVelNED[3] = {0,0,0};
    double initHeading;
    
    NavData.q_B_NED[0] = 1.0;
    NavData.q_B_NED[1] = 0.0;
    NavData.q_B_NED[2] = 0.0;
    NavData.q_B_NED[3] = 0.0;
    
    initPosition[0] = initialPosition[0]*degree2radian;
    initPosition[1] = initialPosition[1]*degree2radian;
    initPosition[2] = initialPosition[2];
    initHeading = initialHeading * degree2radian;
    FsNavigation_setNED(initPosition, initVelNED, initHeading, true);
    
    NavData.state = Calibration;
    
    // Variables
    for (int i=0; i<3; i++)
    {
        gyroError[i] = NULL;
        accelError[i] = NULL;
    }
    gyroVar = 0.0;
    accelVar = 0.0;
    
    // Calibration
    calCounter    = 0;
    maxCalCounter = 100;

    navSetup = true;
}

void FsNavigation_performNavigation( double &navDt )
{
    // Determine Nav State
    if (imuCalibrated) { NavData.state = INS; }
    
    if ( (NavData.state != Calibration) && (navSetup) )
    {
        updateGravity();
        
        // Upate Attitude
        performARHS(navDt);
        
        // Update full Navigation solution
        performINS(navDt);
        
#ifdef SIMULATION
        // Update Navigation Truth
        updateTruth();
#endif
        NavData.timestamp = getTime();
    }
    FsImu_zeroDelta();
}

void updateGravity()
{
    double hCenter;
    hCenter = RE + NavData.position[2];
    NavData.gravity = GMe/(hCenter*hCenter);
}

void performARHS( double &navDt )
{
    // Update quaternion using gyroscope
    gyroUpdate(navDt);
    
    // Apply attitude correction using accelerometer
    compFilter();
}

void gyroUpdate( double &navDt)
{
    // Variables
    double dThetaMag;
    double qMag;
    double unitDir[3];
    double dqGyro[4];
    double qOld[4];
    double dTheta[3];
    
    // Store quaternion
    for (int i=0; i<4; i++)
    { qOld[i] = NavData.q_B_NED[i]; }
    
    // Apply Calibration to dTheta
    for (int i=0; i<3; i++)
    dTheta[i] = nav_pIMUdata->dTheta[i]*degree2radian - NavData.gyroBias[i]*navDt;

    // Angle change magnitude
    dThetaMag = sqrt( dTheta[0]*dTheta[0] +
                     dTheta[1]*dTheta[1] +
                     dTheta[2]*dTheta[2] );
    
    for (int i=0; i<3; i++)
    {
        if (dThetaMag > 0.00001) { unitDir[i]  = dTheta[i] / dThetaMag; }
        else                     { unitDir[i] = 0.0; }
    }
    
    // Compute change in quaternion
    dqGyro[0] = cos(dThetaMag/2);
    dqGyro[1] = unitDir[0]*sin(dThetaMag/2);
    dqGyro[2] = unitDir[1]*sin(dThetaMag/2);
    dqGyro[3] = unitDir[2]*sin(dThetaMag/2);
    
    // Update quaternion
    quaternionProduct(NavData.q_B_NED, qOld, dqGyro);
    
    // Quarantee unit vector
    qMag = sqrt( NavData.q_B_NED[0]*NavData.q_B_NED[0] +
                NavData.q_B_NED[1]*NavData.q_B_NED[1] +
                NavData.q_B_NED[2]*NavData.q_B_NED[2] +
                NavData.q_B_NED[3]*NavData.q_B_NED[3] );
    
    for (int i=0; i<4; i++)
    { NavData.q_B_NED[i] /= qMag; }
}

void compFilter()
{
    // Variables
    double aMag;
    double dThetaMag;
    double gain = 0.0;
    double unitDir[3];
    double accelBody[3];
    double accelNEDNoGravity[3];
    double accelUnitNED[3];
    double qOld[4];
    double dqAccel[4];
    double absAmag;
    static double predUnc = 0.0;
    double gyroUnc = gyroVar;
    double accUnc = accelVar;
    
    // Store quaternion
    for (int i=0; i<4; i++)
    { qOld[i] = NavData.q_B_NED[i]; }
    
    // Rotate acceleration into NED frame using gyroscope prediction
    for (int i=0; i<3; i++)
    {
        accelBody[i] = nav_pIMUdata->accel[i]*NavData.gravity - NavData.accBias[i];
    }
    FsNavigation_bodyToNED(accelUnitNED, accelBody);
    
    accelNEDNoGravity[0] = accelUnitNED[0];
    accelNEDNoGravity[1] = accelUnitNED[1];
    accelNEDNoGravity[2] = accelUnitNED[2] + NavData.gravity;
    
    // Magnitudes and Unit Vectors
    // Acceleration magnitude without gravity
    absAmag = fabs(
                sqrt( accelNEDNoGravity[0]*accelNEDNoGravity[0] +
                accelNEDNoGravity[1]*accelNEDNoGravity[1] +
                accelNEDNoGravity[2]*accelNEDNoGravity[2] )
                );
    
    // Acceleration magnitude with gravity
    aMag = sqrt( accelUnitNED[0]*accelUnitNED[0] +
                accelUnitNED[1]*accelUnitNED[1] +
                accelUnitNED[2]*accelUnitNED[2] );
    
    for (int i=0; i<3; i++)
    {
        if (aMag > 0.01) { accelUnitNED[i]  = accelUnitNED[i] / aMag; }
        else             { accelUnitNED[i] = 0.0; }
    }
    
    // Update uncertainties
    predUnc = predUnc + gyroUnc;
    accUnc += absAmag * 40.0 * accUnc;
    
    // Update gain
    if (predUnc != 0.0 && accUnc != 0.0 && absAmag < 0.1*NavData.gravity)
    {
        gain = predUnc/(predUnc + accUnc);
    }
    
    // Update prediction uncertainty
    predUnc -= gain*predUnc;
    
    // Accelerometer Angle Correction
    dThetaMag = gain * acos( -accelUnitNED[2] );
    
    // Accelerometer Correction Direction
    unitDir[0] = -accelUnitNED[1];
    unitDir[1] = accelUnitNED[0];
    unitDir[2] = 0;
    
    // Compute change in quaternion
    dqAccel[0] = cos(dThetaMag/2);
    dqAccel[1] = unitDir[0]*sin(dThetaMag/2);
    dqAccel[2] = unitDir[1]*sin(dThetaMag/2);
    dqAccel[3] = unitDir[2]*sin(dThetaMag/2);
    
    // Apply accelerometer correction
    quaternionProduct(NavData.q_B_NED, qOld, dqAccel);
}

void performINS( double &navDt )
{
    union insArraysGroup1 {
        double bodyRates[3];
        double gravityNED[3];
        double dVelocity[3];
        double dPosition[3];
    };
    union insArraysGroup2 {
        double gravityBody[3];
        double dVelocityNED[3];
    };
    
    insArraysGroup1 a1;
    insArraysGroup2 a2;
    
    // Euler Angles
    updateEulerAngles();
    
    // Body Rates
    for (int i=0; i<3; i++)
    {
        a1.bodyRates[i] = nav_pIMUdata->gyro[i]*degree2radian - NavData.gyroBias[i];
    }
    
    // Gravity
    a1.gravityNED[0] = 0.0;
    a1.gravityNED[1] = 0.0;
    a1.gravityNED[2] = NavData.gravity;
    
    FsNavigation_NEDToBody(a2.gravityBody, a1.gravityNED);
    
    // Delta Velocity
    for (int i=0; i<3; i++)
    {
        // correct for gravity and bias
        a1.dVelocity[i] = nav_pIMUdata->dVelocity[i]*NavData.gravity - NavData.accBias[i]*navDt + a2.gravityBody[i]*navDt;
    }
    // Body Velocity
    for (int i=0; i<3; i++)
    {
        NavData.velBody[i] += a1.dVelocity[i];
    }
    
    // NED Velocity
    FsNavigation_bodyToNED(a2.dVelocityNED, a1.dVelocity);
    FsNavigation_bodyToNED(NavData.velNED, NavData.velBody);
    
    //Position
    for (int i=0; i<3; i++)
    {
        a1.dPosition[i] =  NavData.velNED[i]*navDt + 0.5*a2.dVelocityNED[i]*navDt;
    }
    
    // Latitude, Longitude, Altitude
    NavData.position[0] = NavData.position[0] + a1.dPosition[0]/(NavData.position[2] + RE);
    NavData.position[1] = NavData.position[1] + a1.dPosition[1]/(NavData.position[2] + RE);
    NavData.position[2] = NavData.position[2] - a1.dPosition[2];
}

void FsNavigation_performGPSPVTUpdate(double* gps_LLA, double* gps_velNED, double gps_heading, double gps_timestamp)
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    NavData.state = GPS;
    
    if (!NavData.initNED)
    {
        FsNavigation_setNED(gps_LLA, gps_velNED, gps_heading);
        return;
    }
}

void FsNavigation_performBarometerUpdate(barometerType* baroData)
{
    
}

void FsNavigation_calibrateIMU()
{
    static bool firstTime = true;
    
    if (imuCalibrated) { return; }
    double tempAcc;
    double tempGyro;
    
    if (firstTime)
    {
        for (int i=0; i<3; i++)
        {
            gyroError[i] = new SensorErrorType;
            accelError[i] = new SensorErrorType;
        }
        firstTime = false;
    }

    // Get Sum
    for (int i=0; i<3; i++)
    {
        // Gyroscope
        tempGyro = nav_pIMUdata->gyro[i] * degree2radian;
        gyroError[i]->sum    += tempGyro;
        gyroError[i]->sumSqr += tempGyro * tempGyro;
        if (nav_pIMUdata->gyro[i] > gyroError[i]->max) { gyroError[i]->max =  tempGyro; }
        if (nav_pIMUdata->gyro[i] < gyroError[i]->min) { gyroError[i]->min =  tempGyro; }
        
        // Accelerometer
        if (i==2) { tempAcc = (nav_pIMUdata->accel[i] + 1.0) * NavData.gravity; }
        else { tempAcc = nav_pIMUdata->accel[i] * NavData.gravity; }
        
        accelError[i]->sum    += tempAcc;
        accelError[i]->sumSqr += tempAcc * tempAcc;
        if (tempAcc > accelError[i]->max) { accelError[i]->max =  tempAcc; }
        if (tempAcc < accelError[i]->min) { accelError[i]->min =  tempAcc; }
    }
    
    calCounter++;
    
    // Compute statistics
    if (calCounter >= maxCalCounter)
    {
        for (int i=0; i<3; i++)
        {
            gyroError[i]->mean = gyroError[i]->sum / maxCalCounter;
            gyroError[i]->variance = gyroError[i]->sumSqr/maxCalCounter - gyroError[i]->mean*gyroError[i]->mean;
            gyroError[i]->std = sqrt(gyroError[i]->variance);
            
            accelError[i]->mean = accelError[i]->sum / maxCalCounter;
            accelError[i]->variance = accelError[i]->sumSqr/maxCalCounter - accelError[i]->mean*accelError[i]->mean;
            accelError[i]->std = sqrt(accelError[i]->variance);
            
            NavData.gyroBias[i] = gyroError[i]->mean;
            NavData.accBias[i]  = accelError[i]->mean;
        }
        
        gyroVar = sqrt( gyroError[0]->variance*gyroError[0]->variance
                        + gyroError[0]->variance*gyroError[0]->variance
                        + gyroError[0]->variance*gyroError[0]->variance );
        accelVar = sqrt( accelError[0]->variance*accelError[0]->variance
                       + accelError[0]->variance*accelError[0]->variance
                       + accelError[0]->variance*accelError[0]->variance );
        
        for (int i=0; i<3; i++)
        {
            delete gyroError[i];
            delete accelError[i];
        }
        
        imuCalibrated = true;
    }
}

NavType* FsNavigation_getNavData(bool useTruth)
{
#ifdef SIMULATION
    if (useTruth) { return &truthNavData; }
    else { return &NavData; }
#else
    return &NavData;
#endif
}

#ifdef SIMULATION
NavType* FsNavigation_getNavError()
{
    return &NavError;
}
#endif

NavState FsNavigation_getNavState()
{
    return NavData.state;
}

#ifdef SIMULATION
void FsNavigation_setSimulationModels(ModelMap* pMap)
{
    pDyn  = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pAtmo = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
}
#endif

void FsNavigation_setIMUdata(IMUtype* pIMUdataIn)
{
    nav_pIMUdata = pIMUdataIn;
}

void FsNavigation_setNED(double* LLA, double* velNED, double heading, bool bypassInitFlag)
{
    double yawDelta;
    double qOld[4];
    double qYawDelta[4];
    
    // Save previous quaternion
    for (int i=0; i<4; i++)
    {
        qOld[i] = NavData.q_B_NED[i];
    }
    
    // Update position and velocity
    for (int i=0; i<3; i++)
    {
        NavData.position[i] = LLA[i];
        NavData.velNED[i] = velNED[i];
    }
    
    // Adjust quaternion for heading
    yawDelta = heading - NavData.eulerAngles[2];
    
    qYawDelta[0] = cos(yawDelta/2);
    qYawDelta[1] = 0.0;
    qYawDelta[2] = 0.0;
    qYawDelta[3] = sin(yawDelta/2);
    
    quaternionProduct(NavData.q_B_NED, qOld, qYawDelta);
    updateEulerAngles();
    
    if (!bypassInitFlag)
    {
        NavData.initNED = true;
    }
}

#ifdef SIMULATION
void updateTruth()
{
    if (pDyn)
    {
        truthNavData.position[0] = pDyn->getPosBody()[0];
        truthNavData.position[1] = pDyn->getPosBody()[1];
        truthNavData.position[2] = pDyn->getPosBody()[2];
        
        truthNavData.velBody[0] = pDyn->getVelBody()[0].mps();
        truthNavData.velBody[1] = pDyn->getVelBody()[1].mps();
        truthNavData.velBody[2] = pDyn->getVelBody()[2].mps();
        
        truthNavData.velNED[0] = pDyn->getVelNED()[0].mps();
        truthNavData.velNED[1] = pDyn->getVelNED()[1].mps();
        truthNavData.velNED[2] = pDyn->getVelNED()[2].mps();

        truthNavData.eulerAngles[0] = pDyn->getEulerAngles()[0].rad();
        truthNavData.eulerAngles[1] = pDyn->getEulerAngles()[1].rad();
        truthNavData.eulerAngles[2] = pDyn->getEulerAngles()[2].rad();
        
        truthNavData.q_B_NED[0] = pDyn->get_q_B_NED()[0];
        truthNavData.q_B_NED[1] = pDyn->get_q_B_NED()[1];
        truthNavData.q_B_NED[2] = pDyn->get_q_B_NED()[2];
        truthNavData.q_B_NED[3] = pDyn->get_q_B_NED()[3];
    }
    
    for (int i=0; i<3; i++)
    {
        NavError.position[i]    = truthNavData.position[i]     - NavData.position[i];
        NavError.velNED[i]      = truthNavData.velNED[i]       - NavData.velNED[i];
        NavError.velBody[i]     = truthNavData.velBody[i]      - NavData.velBody[i];
        NavError.eulerAngles[i] = (truthNavData.eulerAngles[i] - NavData.eulerAngles[i]) * radian2degree;
        NavError.q_B_NED[i]     = truthNavData.q_B_NED[i]      - NavData.q_B_NED[i];
    }
    NavError.position[0] *= radian2degree;
    NavError.position[1] *= radian2degree;
    
    NavError.q_B_NED[3] = truthNavData.q_B_NED[3] - NavData.q_B_NED[3];
}
#endif

inline void FsNavigation_bodyToNED(double* vNED, double* vB)
{
    // vNED = q_B_NED * vB * q_B_NED'
    double q1 =  NavData.q_B_NED[0];
    double q2 = -NavData.q_B_NED[1];
    double q3 = -NavData.q_B_NED[2];
    double q4 = -NavData.q_B_NED[3];
    
    vNED[0] = (2*q1*q1-1+2*q2*q2)*vB[0] + 2*(q2*q3 + q1*q4)  *vB[1] + 2*(q2*q4 - q1*q3)  *vB[2];
    vNED[1] = 2*(q2*q3 - q1*q4)  *vB[0] + (2*q1*q1-1+2*q3*q3)*vB[1] + 2*(q3*q4 + q1*q2)  *vB[2];
    vNED[2] = 2*(q2*q4 + q1*q3)  *vB[0] + 2*(q3*q4 - q1*q2)  *vB[1] + (2*q1*q1-1+2*q4*q4)*vB[2];
}

inline void FsNavigation_NEDToBody(double* vB, double* vNED)
{
    // vB = q_B_NED' * vNED * q_B_NED
    double q1 = NavData.q_B_NED[0];
    double q2 = NavData.q_B_NED[1];
    double q3 = NavData.q_B_NED[2];
    double q4 = NavData.q_B_NED[3];
    
    vB[0] = (2*q1*q1-1+2*q2*q2)*vNED[0] + 2*(q2*q3 + q1*q4)  *vNED[1] + 2*(q2*q4 - q1*q3)  *vNED[2];
    vB[1] = 2*(q2*q3 - q1*q4)  *vNED[0] + (2*q1*q1-1+2*q3*q3)*vNED[1] + 2*(q3*q4 + q1*q2)  *vNED[2];
    vB[2] = 2*(q2*q4 + q1*q3)  *vNED[0] + 2*(q3*q4 - q1*q2)  *vNED[1] + (2*q1*q1-1+2*q4*q4)*vNED[2];
}

void FsNavigation_bodyToLL(double* vLL, double* vBody)
{
    double sr = sin(NavData.eulerAngles[0]);
    double cr = cos(NavData.eulerAngles[0]);
    double sp = sin(NavData.eulerAngles[1]);
    double cp = cos(NavData.eulerAngles[1]);
    
    vLL[0] =  cp*vBody[0] + sp*sr*vBody[1]  + sp*cr*vBody[2];
    vLL[1] =                   cr*vBody[1]  -    sr*vBody[2];
    vLL[2] = -sp*vBody[0] + cp*sr*vBody[1]  + cp*cr*vBody[2];
}

inline void quaternionProduct(double *product, double *q1, double *q2)
{
    product[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    product[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    product[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    product[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

inline void updateEulerAngles()
{
    double* q = NavData.q_B_NED;
    NavData.eulerAngles[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    NavData.eulerAngles[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    NavData.eulerAngles[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
}
