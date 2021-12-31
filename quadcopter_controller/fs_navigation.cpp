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

double gyroCov[3][3];
double accelCov[3][3];

// Simulation Models
#ifdef SIMULATION
    class DynamicsModel*   pDyn = 0;
    class AtmosphereModel* pAtmo = 0;
#endif

// KALMAN FILTER
double PHI[NSTATES][NSTATES];
double P[NSTATES][NSTATES];
double Q[NSTATES][NSTATES];
double stateError[NSTATES];
double diagOnes[NSTATES][NSTATES];

// GPS Correction
double R_GPS[NGPSSTATES][NGPSSTATES];
double H_GPS[NGPSSTATES][NSTATES];
double K_GPS[NSTATES][NGPSSTATES];

// Barometer Correction
double R_BARO[NBAROSTATES][NBAROSTATES];
double H_BARO[NBAROSTATES][NSTATES];
double K_BARO[NSTATES][NBAROSTATES];
double baroRefAltitude;
double baroError[NBAROSTATES];

#ifdef FILTERTEST
double linNavStates[NSTATES];
double navStates[NSTATES];
NavType NavStateError;
NavType linNavStateError;
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
    
    baroRefAltitude = NavData.position[2];
    
    NavData.state = Calibration;
    
    // Variables
    for (int i=0; i<3; i++)
    {
        gyroError[i] = NULL;
        accelError[i] = NULL;
        for (int j=0; j<3; j++)
        {
            gyroCov[i][j]  = 0.0;
            accelCov[i][j] = 0.0;
        }
    }
    gyroVar = 0.0;
    accelVar = 0.0;
    
    // Calibration
    calCounter    = 0;
    maxCalCounter = 100;

    navSetup = true;
    
    // KALMAN FILTER
    for (int i=0; i<NSTATES; i++)
    {
        stateError[i] = 0.0;
        for (int j=0; j<NSTATES; j++)
        {
            P[i][j]          = 0.0;
            Q[i][j]          = 0.0;
            PHI[i][j]        = 0.0;
            
            if (i == j) { diagOnes[i][j] = 1.0; }
            else        { diagOnes[i][j] = 0.0; }
            
            if (i<NGPSSTATES && j<NGPSSTATES) { R_GPS[i][j] = 0.0; }
            if (i<NGPSSTATES)                 { H_GPS[i][j] = 0.0; }
            if (j<NGPSSTATES)                 { K_GPS[i][j] = 0.0; }
            
            if (i<NBAROSTATES && j<NBAROSTATES) { R_BARO[i][j] = 0.0; }
            if (i<NBAROSTATES)                  { H_BARO[i][j] = 0.0; }
            if (j<NBAROSTATES)                  { K_BARO[i][j] = 0.0; }
        }
    }
    
    H_BARO[BARO_ALT][ALT] = 1.0;
    
    // State Covariance (State Uncertainties)
    P[Q0][Q0]   = 0.0;
    P[Q1][Q1]   = 0.0;
    P[Q2][Q2]   = 0.0;
    P[Q3][Q3]   = 0.0;
    P[VN][VN]   = 0.0;
    P[VE][VE]   = 0.0;
    P[VD][VD]   = 0.0;
    P[LAT][LAT] = 0.0;
    P[LON][LON] = 0.0;
    P[ALT][ALT] = 0.0;
    
#ifdef FILTERTEST
    linNavStates[Q0] = NavData.q_B_NED[0];
    linNavStates[Q1] = NavData.q_B_NED[1];
    linNavStates[Q2] = NavData.q_B_NED[2];
    linNavStates[Q3] = NavData.q_B_NED[3];
    linNavStates[VN] = NavData.velNED[0];
    linNavStates[VE] = NavData.velNED[1];
    linNavStates[VD] = NavData.velNED[2];
    linNavStates[LAT] = NavData.position[0];
    linNavStates[LON] = NavData.position[1];
    linNavStates[ALT] = NavData.position[2];
    
    navStates[Q0] = NavData.q_B_NED[0];
    navStates[Q1] = NavData.q_B_NED[1];
    navStates[Q2] = NavData.q_B_NED[2];
    navStates[Q3] = NavData.q_B_NED[3];
    navStates[VN] = NavData.velNED[0];
    navStates[VE] = NavData.velNED[1];
    navStates[VD] = NavData.velNED[2];
    navStates[LAT] = NavData.position[0];
    navStates[LON] = NavData.position[1];
    navStates[ALT] = NavData.position[2];
#endif
}

void FsNavigation_performNavigation( double &navDt )
{
    if (!navSetup) { return; }
    
    // Apply sensor updates
    if (NavData.state == BaroUpdate || NavData.state == GPSUpdate)
    {
        applyCorrections();
        NavData.state = INS;
    }
    
    // Inertial Navigation
    if (NavData.state == INS)
    {
        updateGravity();
        
        // Update Attitude
        performARHS(navDt);
        
        // Update full Navigation solution
        performINS(navDt);
        
        // Propogate Covariance Matrix
        propogateVariance(navDt);
        
#ifdef SIMULATION
        // Update Navigation Truth
        updateTruth();
#endif
        NavData.timestamp = getTime();
        
#ifdef FILTERTEST
        nonlinearStateModel(navDt);
        compareNonlinearStates();
#endif
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
    
    // Store quaternion
    for (int i=0; i<4; i++)
    { qOld[i] = NavData.q_B_NED[i]; }
    
    // Apply Calibration to dTheta
    for (int i=0; i<3; i++)
    NavData.stateInputs.dTheta[i] = nav_pIMUdata->dTheta[i]*degree2radian - NavData.gyroBias[i]*navDt;

    // Angle change magnitude
    dThetaMag = sqrt( NavData.stateInputs.dTheta[0]*NavData.stateInputs.dTheta[0] +
                     NavData.stateInputs.dTheta[1]*NavData.stateInputs.dTheta[1] +
                     NavData.stateInputs.dTheta[2]*NavData.stateInputs.dTheta[2] );
    
    for (int i=0; i<3; i++)
    {
        if (dThetaMag > 0.00001) { unitDir[i]  = NavData.stateInputs.dTheta[i] / dThetaMag; }
        else                     { unitDir[i] = 0.0; }
    }
    
    // Compute change in quaternion
    dqGyro[0] = cos(dThetaMag/2.0);
    dqGyro[1] = unitDir[0]*sin(dThetaMag/2.0);
    dqGyro[2] = unitDir[1]*sin(dThetaMag/2.0);
    dqGyro[3] = unitDir[2]*sin(dThetaMag/2.0);
    
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
        accelBody[i] = nav_pIMUdata->accel[i] - NavData.accBias[i];
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
    dqAccel[0] = cos(dThetaMag/2.0);
    dqAccel[1] = unitDir[0]*sin(dThetaMag/2.0);
    dqAccel[2] = unitDir[1]*sin(dThetaMag/2.0);
    dqAccel[3] = unitDir[2]*sin(dThetaMag/2.0);
    
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
        NavData.stateInputs.dVelocity[i] = nav_pIMUdata->dVelocity[i] - NavData.accBias[i]*navDt + a2.gravityBody[i]*navDt;
    }
    // Body Velocity
    for (int i=0; i<3; i++)
    {
        NavData.velBody[i] += NavData.stateInputs.dVelocity[i];
    }
    
    // NED Velocity
    FsNavigation_bodyToNED(a2.dVelocityNED, NavData.stateInputs.dVelocity);
    FsNavigation_bodyToNED(NavData.velNED, NavData.velBody);

    //Position
    for (int i=0; i<3; i++)
    {
        a1.dPosition[i] =  NavData.velNED[i]*navDt;// + 0.5*a2.dVelocityNED[i]*navDt;
    }
    
    // Latitude, Longitude, Altitude
    NavData.position[0] = NavData.position[0] + a1.dPosition[0]/(NavData.position[2] + RE);
    NavData.position[1] = NavData.position[1] + a1.dPosition[1]/( (NavData.position[2] + RE)*cos(NavData.position[0]) );
    NavData.position[2] = NavData.position[2] - a1.dPosition[2];
}


void propogateVariance( double &navDt )
{
    // Intermidiate variables
    double q00;
    double q01;
    double q02;
    double q03;
    double q11;
    double q12;
    double q13;
    double q22;
    double q23;
    double q33;
    double dThetaMag;
    double unitTheta[3] = {0.0};
    double c2;
    double s2;
    double* q = NavData.q_B_NED;
    double invR;
    double invR2;
    double secLat;
    double Qc[NSTATES][NSTATES] = {0.0};
    double PHI_V[3][3] = {0.0};
    double Qc_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHI_Qc_PHItrans[NSTATES][NSTATES] = {0.0};
    double prevLinNavStates[NSTATES] = {0.0};
    double PHItrans[NSTATES][NSTATES] = {0.0};
    double P_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHI_P_PHItrans[NSTATES][NSTATES] = {0.0};
    
    q00 = q[0]*q[0];
    q01 = q[0]*q[1];
    q02 = q[0]*q[2];
    q03 = q[0]*q[3];
    q11 = q[1]*q[1];
    q12 = q[1]*q[2];
    q13 = q[1]*q[3];
    q22 = q[2]*q[2];
    q23 = q[2]*q[3];
    q33 = q[3]*q[3];
    
    PHI_V[0][0] = 2.0*q00 - 1.0 + 2.0*q11; PHI_V[0][1] =      2.0*(q12 - q03)     ;  PHI_V[0][2] = 2.0*(q13 + q02);
    PHI_V[1][0] =     2.0*(q12 + q03)    ; PHI_V[1][1] = (2.0*q00 - 1.0 + 2.0*q22);  PHI_V[1][2] = 2.0*(q23 - q01);
    PHI_V[2][0] =     2.0*(q13 - q02)    ; PHI_V[2][1] =      2.0*(q23 + q01)     ;  PHI_V[2][2] = (2.0*q00 - 1.0 + 2.0*q33);
    
    dThetaMag = sqrt( NavData.stateInputs.dTheta[0]*NavData.stateInputs.dTheta[0] +
                     NavData.stateInputs.dTheta[1]*NavData.stateInputs.dTheta[1] +
                     NavData.stateInputs.dTheta[2]*NavData.stateInputs.dTheta[2] );
    
    for (int i=0; i<3; i++)
    {
        if (dThetaMag > 0.00001) { unitTheta[i]  = NavData.stateInputs.dTheta[i] / dThetaMag; }
        else                     { unitTheta[i] = 0.0; }
    }
    
    c2 = cos(dThetaMag / 2.0);
    s2 = sin(dThetaMag / 2.0);
    
    invR   = 1.0/(RE + NavData.position[2]);
    invR2  = invR*invR;
    secLat = 1.0 / cos(NavData.position[0]);
    
    // Update State Transition Matrix
    // Q0
    PHI[Q0][Q0] = c2;
    PHI[Q0][Q1] = -unitTheta[0] * s2;
    PHI[Q0][Q2] = -unitTheta[1] * s2;
    PHI[Q0][Q3] = -unitTheta[2] * s2;
    // Q1
    PHI[Q1][Q0] = unitTheta[0] * s2;
    PHI[Q1][Q1] = c2;
    PHI[Q1][Q2] = unitTheta[2] * s2;
    PHI[Q1][Q3] = -unitTheta[1] * s2;
    // Q2
    PHI[Q2][Q0] = unitTheta[1] * s2;
    PHI[Q2][Q1] = -unitTheta[2] * s2;
    PHI[Q2][Q2] = c2;
    PHI[Q2][Q3] = unitTheta[0] * s2;
    // Q3
    PHI[Q3][Q0] = unitTheta[2] * s2;
    PHI[Q3][Q1] = unitTheta[1] * s2;
    PHI[Q3][Q2] = -unitTheta[0] * s2;
    PHI[Q3][Q3] = c2;
    //VN
    PHI[VN][Q0] = 4.0*q[0]*NavData.stateInputs.dVelocity[0] - 2.0*q[3]*NavData.stateInputs.dVelocity[1] + 2.0*q[2]*NavData.stateInputs.dVelocity[2];
    PHI[VN][Q1] = 4.0*q[1]*NavData.stateInputs.dVelocity[0] + 2.0*q[2]*NavData.stateInputs.dVelocity[1] + 2.0*q[3]*NavData.stateInputs.dVelocity[2];
    PHI[VN][Q2] = 2.0*q[1]*NavData.stateInputs.dVelocity[1] + 2.0*q[0]*NavData.stateInputs.dVelocity[2];
    PHI[VN][Q3] = -2.0*q[0]*NavData.stateInputs.dVelocity[1] + 2.0*q[1]*NavData.stateInputs.dVelocity[2];
    PHI[VN][VN] = 1.0;
    //VE
    PHI[VE][Q0] = 2.0*q[3]*NavData.stateInputs.dVelocity[0] + 4.0*q[0]*NavData.stateInputs.dVelocity[1] - 2.0*q[1]*NavData.stateInputs.dVelocity[2];
    PHI[VE][Q1] = 2.0*q[2]*NavData.stateInputs.dVelocity[0] - 2.0*q[0]*NavData.stateInputs.dVelocity[2];
    PHI[VE][Q2] = 2.0*q[1]*NavData.stateInputs.dVelocity[0] + 4.0*q[2]*NavData.stateInputs.dVelocity[1] + 2.0*q[3]*NavData.stateInputs.dVelocity[2];
    PHI[VE][Q3] = 2.0*q[0]*NavData.stateInputs.dVelocity[0] + 2.0*q[2]*NavData.stateInputs.dVelocity[2];
    PHI[VE][VE] = 1.0;
    //VD
    PHI[VD][Q0] = -2.0*q[2]*NavData.stateInputs.dVelocity[0] + 2.0*q[1]*NavData.stateInputs.dVelocity[1] + 4.0*q[0]*NavData.stateInputs.dVelocity[2];
    PHI[VD][Q1] = 2.0*q[3]*NavData.stateInputs.dVelocity[0] + 2.0*q[0]*NavData.stateInputs.dVelocity[1];
    PHI[VD][Q2] = -2.0*q[0]*NavData.stateInputs.dVelocity[0] + 2.0*q[3]*NavData.stateInputs.dVelocity[1];
    PHI[VD][Q3] = 2.0*q[1]*NavData.stateInputs.dVelocity[0] + 2.0*q[2]*NavData.stateInputs.dVelocity[1] + 4.0*q[3]*NavData.stateInputs.dVelocity[2];
    PHI[VD][VD] = 1.0;
    //LAT
    PHI[LAT][VN] = invR * navDt;
    PHI[LAT][LAT] = 1.0;
    PHI[LAT][ALT] = -2.0 * NavData.velNED[0] * invR2 * navDt;
    //LON
    PHI[LON][VE] = invR * secLat * navDt;
    PHI[LON][LAT] = NavData.velNED[1] * tan(NavData.position[0]) * invR * secLat * navDt;
    PHI[LON][LON] = 1.0;
    PHI[LON][ALT] = -2.0 * NavData.velNED[1] * invR2 * secLat * navDt;
    //ALT
    PHI[ALT][VD] = -navDt;
    PHI[ALT][ALT] = 1.0;
    
    math_mtran(*PHItrans, *PHI, NSTATES, NSTATES);
    
    // Update Process Noise
    Qc[Q0][Q0] = 0.25*q11*gyroCov[X][X] + 0.25*q22*gyroCov[Y][Y] + 0.25*q33*gyroCov[Z][Z] + 0.5*q12*gyroCov[X][Y] + 0.5*q23*gyroCov[Y][Z] + q13*gyroCov[X][Z];
    
    Qc[Q1][Q1] = 0.25*q00*gyroCov[X][X] + 0.25*q33*gyroCov[Y][Y] + 0.25*q22*gyroCov[Z][Z] - 0.5*q03*gyroCov[X][Y] - 0.5*q23*gyroCov[Y][Z] + q02*gyroCov[X][Z];
    
    Qc[Q2][Q2] = 0.25*q33*gyroCov[X][X] + 0.25*q00*gyroCov[Y][Y] + 0.25*q11*gyroCov[Z][Z] + 0.5*q03*gyroCov[X][Y] - 0.5*q01*gyroCov[Y][Z] - q13*gyroCov[X][Z];
    
    Qc[Q3][Q3] = 0.25*q22*gyroCov[X][X] + 0.25*q11*gyroCov[Y][Y] + 0.25*q00*gyroCov[Z][Z] - 0.5*q12*gyroCov[X][Y] + 0.5*q01*gyroCov[Y][Z] - q02*gyroCov[X][Z];
    
    Qc[VN][VN] = PHI_V[0][0]*PHI_V[0][0]*accelCov[X][X] + PHI_V[0][1]*PHI_V[0][1]*accelCov[Y][Y] + PHI_V[0][2]*PHI_V[0][2]*accelCov[Z][Z] + PHI_V[0][0]*PHI_V[0][1]*accelCov[X][Y] + PHI_V[0][1]*PHI_V[0][2]*accelCov[Y][Z] + PHI_V[0][0]*PHI_V[0][2]*accelCov[X][Z];
    
    Qc[VE][VE] = PHI_V[1][0]*PHI_V[1][0]*accelCov[X][X] + PHI_V[1][1]*PHI_V[1][1]*accelCov[Y][Y] + PHI_V[1][2]*PHI_V[1][2]*accelCov[Z][Z] + PHI_V[1][0]*PHI_V[1][1]*accelCov[X][Y] + PHI_V[1][1]*PHI_V[1][2]*accelCov[Y][Z] + PHI_V[1][0]*PHI_V[1][2]*accelCov[X][Z];
    
    Qc[VD][VD] = PHI_V[2][0]*PHI_V[2][0]*accelCov[X][X] + PHI_V[2][1]*PHI_V[2][1]*accelCov[Y][Y] + PHI_V[2][2]*PHI_V[2][2]*accelCov[Z][Z] + PHI_V[2][0]*PHI_V[2][1]*accelCov[X][Y] + PHI_V[2][1]*PHI_V[2][2]*accelCov[Y][Z] + PHI_V[2][0]*PHI_V[2][2]*accelCov[X][Z];
    
    Qc[LAT][LAT] = 0.0;
    Qc[LON][LON] = 0.0;
    Qc[ALT][ALT] = 0.0;
    
    //Q = PHI*Qc*PHI^T dt;
    math_mmult(*Qc_PHItrans, *Qc, NSTATES, NSTATES, *PHItrans, NSTATES, NSTATES);
    math_mmult(*PHI_Qc_PHItrans, *PHI, NSTATES, NSTATES, *Qc_PHItrans, NSTATES, NSTATES);
    math_mgain(*Q, *PHI_Qc_PHItrans, navDt, NSTATES, NSTATES);
    
    // Propogate Covariance
    //P = PHI*P*PHI^T + Q;
    math_mmult(*P_PHItrans, *P, NSTATES, NSTATES, *PHItrans, NSTATES, NSTATES);
    math_mmult(*PHI_P_PHItrans, *PHI, NSTATES, NSTATES, *P_PHItrans, NSTATES, NSTATES);
    math_madd(*P, *PHI_P_PHItrans, *Q, NSTATES, NSTATES);
    
#ifdef FILTERTEST
    for (int i=0; i<NSTATES; i++)
    {
        prevLinNavStates[i] = linNavStates[i];
    }
    
    // xk1 = f(xk, u)
    // xk1 = f(xk0, u0) + dfdx * (xk - xk0) + dfdu * (u - u0)
    // 
    // linNavStates = PHI * prevLinNavStates
    math_mmult(linNavStates, *PHI, prevLinNavStates, NSTATES, NSTATES);
#endif
}

void applyCorrections()
{
    NavData.q_B_NED[0] += stateError[Q0];
    NavData.q_B_NED[1] += stateError[Q1];
    NavData.q_B_NED[2] += stateError[Q2];
    NavData.q_B_NED[3] += stateError[Q3];
    
    NavData.velNED[0] += stateError[VN];
    NavData.velNED[1] += stateError[VE];
    NavData.velNED[2] += stateError[VD];
    
    NavData.position[0] += stateError[LAT];
    NavData.position[1] += stateError[LON];
    NavData.position[2] += stateError[ALT];
    
    updateEulerAngles();
    
    for (int i = 0; i < NSTATES; i++)
    {
        //stateError[i] = 0.0;
    }
}

void FsNavigation_performGPSPVTUpdate(double* gps_LLA, double* gps_velNED, double gps_heading, double gps_timestamp)
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    if(NavData.state != INS)
    {
        display("FsNavigation_performGPSPVTUpdate: WARNING. Not in INS state");
        //return;
    }
    
    NavData.state = GPSUpdate;
    
    if (!NavData.initNED)
    {
        // Store previous Nav altitude solution
        double prevAltitude = NavData.position[2];
        
        // Initialize NED solution with GPS
        FsNavigation_setNED(gps_LLA, gps_velNED, gps_heading);
        
        // Set new barometer reference altitude
        baroRefAltitude = baroRefAltitude + (prevAltitude - NavData.position[2]);
        
        return;
    }
}

void FsNavigation_performBarometerUpdate(barometerType* baroData)
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    if(NavData.state != INS)
    {
        display("FsNavigation_performBarometerUpdate: WARNING. Not in INS state");
        //return;
    }
    
    NavData.state = BaroUpdate;
    
    double HBAROT[NSTATES][NBAROSTATES];
    double P_HBAROT[NBAROSTATES][NSTATES];
    double HBARO_P_HBAROT[NBAROSTATES][NBAROSTATES];
    double HBARO_P_HBAROT_R[NBAROSTATES][NBAROSTATES];
    double inv_HBARO_P_HBAROT_R[NBAROSTATES][NBAROSTATES];
    double HBAROT_inv_HBARO_P_HBAROT_R[NSTATES][NBAROSTATES];
    
    double KBARO_HBARO[NSTATES][NSTATES];
    double diagOnes_KBARO_HBARO[NSTATES][NSTATES];
    double Pprev[NSTATES][NSTATES];
    
    for (int i = 0; i < NSTATES; i++)
    {
        for (int j = 0; j < NSTATES; j++)
        {
            Pprev[i][j] = P[i][j];
        }
    }
    
    R_BARO[BARO_ALT][BARO_ALT] = FsBarometer_getAltitudeVariance();
    
    baroError[BARO_ALT] = baroData->altitude + baroRefAltitude - NavData.position[2];

    // NSTATES x NMESAUREMENTS
    // 10x1   = (10x10) * (10x1)  *    (  1x10  * 10x10 *  10x1   +   1x1 )
    // K_BARO =    P    * H_BARO' * inv( H_BARO *   P   * H_BARO' + R_BARO);
    math_mtran(*HBAROT, *H_BARO, NSTATES, NBAROSTATES);
    math_mmult(*P_HBAROT, *P, NSTATES, NSTATES, *HBAROT, NSTATES, NBAROSTATES);
    math_mmult(*HBARO_P_HBAROT, *H_BARO, NBAROSTATES, NSTATES, *P_HBAROT, NSTATES, NBAROSTATES);
    math_madd(*HBARO_P_HBAROT_R, *HBARO_P_HBAROT, *R_BARO, NBAROSTATES, NBAROSTATES);
    math_minv(*inv_HBARO_P_HBAROT_R, *HBARO_P_HBAROT_R, NBAROSTATES);
    math_mmult(*HBAROT_inv_HBARO_P_HBAROT_R, *HBAROT, NSTATES, NBAROSTATES, *inv_HBARO_P_HBAROT_R, NBAROSTATES, NBAROSTATES);
    math_mmult(*K_BARO, *P, NSTATES, NSTATES, *HBAROT_inv_HBARO_P_HBAROT_R, NSTATES, NBAROSTATES);

    //    10x1    =  10x1  *   1x1
    // stateError = K_Baro * baroError
    math_mmult(stateError, *K_BARO, baroError, NSTATES, NBAROSTATES);
    
    //10x10 = (10x10 -   10x1 *   1x10) * 10x10
    // P    =  (  I   - K_BARO * H_BARO) *   P
    math_mmult(*KBARO_HBARO, *K_BARO, NSTATES, NBAROSTATES, *H_BARO, NBAROSTATES, NSTATES);
    math_msubtract(*diagOnes_KBARO_HBARO, *diagOnes, *KBARO_HBARO, NSTATES, NSTATES);
    math_mmult(*P, *diagOnes_KBARO_HBARO, NSTATES, NSTATES, *Pprev, NSTATES, NSTATES);
}

#ifdef FILTERTEST
void nonlinearStateModel(double &navDt)
{
    double dTheta[3];
    double dThetaMag;
    double c2;
    double s2;
    double unitTheta[3];
    double dVelocity[3];
    double prevState[NSTATES];
    double q00;
    double q01;
    double q02;
    double q03;
    double q11;
    double q12;
    double q13;
    double q22;
    double q23;
    double q33;
    
    // State Inputs
    for (int i=0; i<3; i++)
    {
        dTheta[i] = NavData.stateInputs.dTheta[i];
        dVelocity[i] = NavData.stateInputs.dVelocity[i];
    }
    
    // Store previous state
    for (int i=0; i<NSTATES; i++)
    {
        prevState[i] = navStates[i];
    }
    
    // Intermidiate variables
    dThetaMag = sqrt( dTheta[0]*dTheta[0] +
                     dTheta[1]*dTheta[1] +
                     dTheta[2]*dTheta[2] );
    for (int i=0; i<3; i++)
    {
        for (int i=0; i<3; i++)
        {
            if (dThetaMag > 0.00001) { unitTheta[i]  = dTheta[i] / dThetaMag; }
            else                     { unitTheta[i] = 0.0; }
        }
    }
    
    c2 = cos(dThetaMag/2.0);
    s2 = sin(dThetaMag/2.0);
    
    q00 = prevState[Q0]*prevState[Q0];
    q01 = prevState[Q0]*prevState[Q1];
    q02 = prevState[Q0]*prevState[Q2];
    q03 = prevState[Q0]*prevState[Q3];
    q11 = prevState[Q1]*prevState[Q1];
    q12 = prevState[Q1]*prevState[Q2];
    q13 = prevState[Q1]*prevState[Q3];
    q22 = prevState[Q2]*prevState[Q2];
    q23 = prevState[Q2]*prevState[Q3];
    q33 = prevState[Q3]*prevState[Q3];
    
    navStates[Q0] = prevState[Q0]*c2              - prevState[Q1]*unitTheta[0]*s2 - prevState[Q2]*unitTheta[1]*s2 - prevState[Q3]*unitTheta[2]*s2;
    navStates[Q1] = prevState[Q0]*unitTheta[0]*s2 + prevState[Q1]*c2              + prevState[Q2]*unitTheta[2]*s2 - prevState[Q3]*unitTheta[1]*s2;
    navStates[Q2] = prevState[Q0]*unitTheta[1]*s2 - prevState[Q1]*unitTheta[2]*s2 + prevState[Q2]*c2              + prevState[Q3]*unitTheta[0]*s2;
    navStates[Q3] = prevState[Q0]*unitTheta[2]*s2 + prevState[Q1]*unitTheta[1]*s2 - prevState[Q2]*unitTheta[0]*s2 + prevState[Q3]*c2;
    navStates[VN] = prevState[VN] + (2.0*q00 - 1.0 + 2.0*q11)*dVelocity[0] + 2.0*(q12 - q03)          *dVelocity[1] + 2.0*(q13 + q02)          *dVelocity[2];
    navStates[VE] = prevState[VE] + 2.0*(q12 + q03)          *dVelocity[0] + (2.0*q00 - 1.0 + 2.0*q22)*dVelocity[1] + 2.0*(q23 - q01)          *dVelocity[2];
    navStates[VD] = prevState[VD] + 2.0*(q13 - q02)          *dVelocity[0] + 2.0*(q23 + q01)          *dVelocity[1] + (2.0*q00 - 1.0 + 2.0*q33)*dVelocity[2];
    navStates[LAT] = prevState[LAT] + prevState[VN] * navDt / (RE + prevState[ALT]);
    navStates[LON] = prevState[LON] + prevState[VE] * navDt / ( (RE + prevState[ALT])*cos(prevState[LAT]) );
    navStates[ALT] = prevState[ALT] - prevState[VD] * navDt;
}

void compareNonlinearStates()
{
    double q[4];
    double eulerAngles[3];
    double linEulerAngles[3];
    
    q[0] = navStates[Q0];
    q[1] = navStates[Q1];
    q[2] = navStates[Q2];
    q[3] = navStates[Q3];
    eulerAngles[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    eulerAngles[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    eulerAngles[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
    
    // Nonlinear Nav State Error
    NavStateError.eulerAngles[0] = (NavData.eulerAngles[0] - eulerAngles[0]) * radian2degree;
    NavStateError.eulerAngles[1] = (NavData.eulerAngles[1] - eulerAngles[1]) * radian2degree;
    NavStateError.eulerAngles[2] = (NavData.eulerAngles[2] - eulerAngles[2]) * radian2degree;
    
    NavStateError.q_B_NED[0] = NavData.q_B_NED[0] - navStates[Q0];
    NavStateError.q_B_NED[1] = NavData.q_B_NED[1] - navStates[Q1];
    NavStateError.q_B_NED[2] = NavData.q_B_NED[2] - navStates[Q2];
    NavStateError.q_B_NED[3] = NavData.q_B_NED[3] - navStates[Q3];
    
    NavStateError.position[0] = (NavData.position[0] - navStates[LAT]) * radian2degree;
    NavStateError.position[1] = (NavData.position[1] - navStates[LON]) * radian2degree;
    NavStateError.position[2] = (NavData.position[2] - navStates[ALT]);
    
    NavStateError.velNED[0] = NavData.velNED[0] - navStates[VN];
    NavStateError.velNED[1] = NavData.velNED[1] - navStates[VE];
    NavStateError.velNED[2] = NavData.velNED[2] - navStates[VD];
    
    // Linear Error
    q[0] = linNavStates[Q0];
    q[1] = linNavStates[Q1];
    q[2] = linNavStates[Q2];
    q[3] = linNavStates[Q3];
    linEulerAngles[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    linEulerAngles[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    linEulerAngles[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
    
    linNavStateError.eulerAngles[0] = (eulerAngles[0] - linEulerAngles[0]) * radian2degree;
    linNavStateError.eulerAngles[1] = (eulerAngles[1] - linEulerAngles[1]) * radian2degree;
    linNavStateError.eulerAngles[2] = (eulerAngles[2] - linEulerAngles[2]) * radian2degree;
    
    linNavStateError.q_B_NED[0] = navStates[Q0] - linNavStates[Q0];
    linNavStateError.q_B_NED[1] = navStates[Q1] - linNavStates[Q1];
    linNavStateError.q_B_NED[2] = navStates[Q2] - linNavStates[Q2];
    linNavStateError.q_B_NED[3] = navStates[Q3] - linNavStates[Q3];
    
    linNavStateError.position[0] = (navStates[LAT] - linNavStates[LAT]) * radian2degree;
    linNavStateError.position[1] = (navStates[LON] - linNavStates[LON]) * radian2degree;
    linNavStateError.position[2] = (navStates[ALT] - linNavStates[ALT]);
    
    linNavStateError.velNED[0] = navStates[VN] - linNavStates[VN];
    linNavStateError.velNED[1] = navStates[VE] - linNavStates[VE];
    linNavStateError.velNED[2] = navStates[VD] - linNavStates[VD];
    /*
    linNavStateError.eulerAngles[0] = (NavData.eulerAngles[0] - linEulerAngles[0]) * radian2degree;
    linNavStateError.eulerAngles[1] = (NavData.eulerAngles[1] - linEulerAngles[1]) * radian2degree;
    linNavStateError.eulerAngles[2] = (NavData.eulerAngles[2] - linEulerAngles[2]) * radian2degree;
    
    linNavStateError.q_B_NED[0] = NavData.q_B_NED[0] - linNavStates[Q0];
    linNavStateError.q_B_NED[1] = NavData.q_B_NED[1] - linNavStates[Q1];
    linNavStateError.q_B_NED[2] = NavData.q_B_NED[2] - linNavStates[Q2];
    linNavStateError.q_B_NED[3] = NavData.q_B_NED[3] - linNavStates[Q3];
    
    linNavStateError.position[0] = (NavData.position[0] - linNavStates[LAT]) * radian2degree;
    linNavStateError.position[1] = (NavData.position[1] - linNavStates[LON]) * radian2degree;
    linNavStateError.position[2] = (NavData.position[2] - linNavStates[ALT]);
    
    linNavStateError.velNED[0] = NavData.velNED[0] - linNavStates[VN];
    linNavStateError.velNED[1] = NavData.velNED[1] - linNavStates[VE];
    linNavStateError.velNED[2] = NavData.velNED[2] - linNavStates[VD];
     */
}

#endif

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
        if (i==2) { tempAcc = nav_pIMUdata->accel[i] + NavData.gravity; }
        else { tempAcc = nav_pIMUdata->accel[i]; }
        
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
            gyroError[i]->mean     = gyroError[i]->sum / maxCalCounter;
            gyroError[i]->variance = gyroError[i]->sumSqr/maxCalCounter - gyroError[i]->mean*gyroError[i]->mean;
            gyroError[i]->std      = sqrt(gyroError[i]->variance);
            
            accelError[i]->mean     = accelError[i]->sum / maxCalCounter;
            accelError[i]->variance = accelError[i]->sumSqr/maxCalCounter - accelError[i]->mean*accelError[i]->mean;
            accelError[i]->std      = sqrt(accelError[i]->variance);
            
            NavData.gyroBias[i] = gyroError[i]->mean;
            NavData.accBias[i]  = accelError[i]->mean;
        }
        
        gyroCov[0][0]  = gyroError[0]->variance;
        gyroCov[1][1]  = gyroError[1]->variance;
        gyroCov[2][2]  = gyroError[2]->variance;
        accelCov[0][0] = accelError[0]->variance;
        accelCov[1][1] = accelError[1]->variance;
        accelCov[2][2] = accelError[2]->variance;
        
        gyroVar = sqrt( gyroError[0]->variance*gyroError[0]->variance
                        + gyroError[1]->variance*gyroError[1]->variance
                        + gyroError[2]->variance*gyroError[2]->variance );
        accelVar = sqrt( accelError[0]->variance*accelError[0]->variance
                       + accelError[1]->variance*accelError[1]->variance
                       + accelError[2]->variance*accelError[2]->variance );
        
        for (int i=0; i<3; i++)
        {
            delete gyroError[i];
            delete accelError[i];
        }
        
        imuCalibrated = true;
        
        NavData.state = INS;
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

double* FsNavigation_getCovariance()
{
    return *P;
}

double* FsNavigation_getProcessNoise()
{
    return *Q;
}

double* FsNavigation_getBaroKalmanGain()
{
    return *K_BARO;
}

double* FsNavigation_getStateError()
{
    return stateError;
}

#ifdef FILTERTEST
NavType* FsNavigation_getNavStateError()
{
    return &NavStateError;
}

NavType* FsNavigation_getLinStateError()
{
    return &linNavStateError;
}
#endif

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

// Math
void math_mgain(double *result, double *matrix, double gain, int nrow, int ncol)
{
    double Aij;
    for (int i=0; i<nrow; i++)
    {
        for (int j=0; j<ncol; j++)
        {
            Aij = *(matrix+i*ncol+j);
            *(result+i*ncol+j) = Aij * gain;
        }
    }
}

void math_madd(double *result, double *A, double *B, int nrows, int ncols)
{
    double Aij;
    double Bij;
    for (int i=0; i<nrows; i++)
    {
        for (int j=0; j<ncols; j++)
        {
            Aij = *(A+i*ncols+j);
            Bij = *(B+i*ncols+j);
            *(result+i*ncols+j) = Aij + Bij;
        }
    }
}

void math_msubtract(double *result, double *A, double *B, int nrows, int ncols)
{
    double Aij;
    double Bij;
    for (int i=0; i<nrows; i++)
    {
        for (int j=0; j<ncols; j++)
        {
            Aij = *(A+i*ncols+j);
            Bij = *(B+i*ncols+j);
            *(result+i*ncols+j) = Aij - Bij;
        }
    }
}

void math_mmult(double* result, double* A, int nrows1, int ncols1, double* B, int nrows2, int ncols2)
{
    int Arow;
    int Acol;
    int Brow;
    int Bcol;
    double Aik;
    double Bkj;
    
    if (ncols1 != nrows2)
    {
        display("fs_navigation::math_mmult: WARNING. Columns of matrix 1 (");
        display(ncols1);
        display(") do not match rows of matrix 2 (");
        display(nrows2);
        display(").\n");
        return;
    }
    
    for(int i = 0; i < nrows1; i++)
    {
        for(int j = 0; j < ncols2; j++)
        {
            // zero sum
            *(result+i*ncols2+j) = 0;
            
            for(int k=0;k<ncols1;k++)
            {
                Arow = i; Acol = k; Brow = k; Bcol = j;
                Aik = *(A+Arow*ncols1+Acol); // A[i][k]
                Bkj = *(B+Brow*ncols2+Bcol); // B[k][j]
                *(result+i*ncols2+j) += Aik*Bkj;      //C[i][j] = A[i][k]*B[k][j]
            }
        }
    }
}

void math_mmult(double *result, double *matrix, double *vector, int nrow, int ncol)
{
    double matrix_ij;
    double vector_j;
    for(int i = 0; i < nrow; i++)
    {
        // zero sum
        *(result+i) = 0;
        
        //result[i] = sum( matrix[i][1:n] )*vector[j]
        for(int j = 0; j < ncol; j++)
        {
            matrix_ij = *(matrix + i*ncol + j);// matrix[i][j]
            vector_j  = *(vector + j);         // vector[j]
            *(result+i) += matrix_ij*vector_j;
        }
    }
}

void math_mtran(double *matrix_t, double *matrix, int nrow_t, int ncol_t)
{
    for(int i=0;i<nrow_t;i++)
    {
        for(int j=0;j<ncol_t;j++)
        {
            *(matrix_t+i*ncol_t+j) = *(matrix+j*nrow_t+i);
        }
    }
}

void math_minv(double *matrix_inv, double *matrix, int n)
{
    double b[n];
    double x[n];
    for (int i = 0; i < n; i++)
    {
        b[i] = 0.0;
        x[i] = 0.0;
    }
    
    for (int icol = 0; icol < n; icol++)
    {
        // matrix_inv[1:n,icol] = A\b
        // where b[icol] = 1 and b[!icol] = 0
        b[icol] = 1.0;
        math_LUdecomp(x, matrix, b, n);
        for (int irow = 0; irow < n; irow++)
        {
            *(matrix_inv+irow*n+icol) = x[irow];
        }
        b[icol] = 0.0;
    }
}

void math_LUdecomp(double *x, double *A, double *b, int n)
{
    // Initialize variables
    double L[n][n];
    double U[n][n];
    double xstar[n];
    double sum;
    
    // Initialize L and U
    for(int i=0;i<n;i++){
        L[i][0] = *(A+i*n+0); //L[i][0] = A[i][0]
        U[i][i] = 1; // U diagonals = 1
        if(i > 0) {
            U[0][i] = *(A+0*n+i)/L[0][0]; // U[0][i] = A[0][i]/L[0][0]
        }
    }
    
    // Compute full L and U matrices
    for(int i=1;i<n;i++){
        // Compute col i of L
        for(int j=i;j<n;j++){
            // j is row of L, col of U
            // i is row of U, col of L
            sum = 0;
            for(int k=0;k<i;k++){
                sum += L[j][k]*U[k][i];
            }
            L[j][i] = *(A+j*n+i) - sum;
        }
        // Compute row i of U
        for(int j=i+1;j<n;j++){
            sum = 0;
            for(int k=0;k<i;k++){
                sum += L[i][k]*U[k][j];
            }
            U[i][j] = (*(A+i*n+j) - sum)/L[i][i];
        }
    }
    
    // Foward substitution L*xstar = b, solve for xstar
    xstar[0] = b[0]/L[0][0];
    for(int i=1;i<n;i++){
        sum = 0;
        for(int k=0;k<i;k++){
            sum += L[i][k]*xstar[k];
        }
        xstar[i] = (b[i] - sum)/L[i][i];
    }
    
    // Backward substitution U*x = xstar, solve for x
    x[n-1] = xstar[n-1]; //x[n-1] = xstar[n-1]/U[n-1][n-1];
    for (int i=n-2;i>-1;i--){
        sum = 0;
        for(int k=i+1;k<n;k++){
            sum += U[i][k]*x[k];
        }
        x[i] = xstar[i] - sum; //x[i] = (xstar[i] - sum)/U[i][i];
    }
}
