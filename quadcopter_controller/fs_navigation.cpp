//
//  fs_navigation.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/20/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_navigation.hpp"
#include "fs_imu.hpp"
#include "fs_controls.hpp"

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

// Ground Align Correction
double R_GROUND[NGROUNDSTATES][NGROUNDSTATES];
double H_GROUND[NGROUNDSTATES][NSTATES];
double K_GROUND[NSTATES][NGROUNDSTATES];

// Accelerometer Correction
double R_ACCEL[NACCELSTATES][NACCELSTATES];
double H_ACCEL[NACCELSTATES][NSTATES];
double K_ACCEL[NSTATES][NACCELSTATES];
double accelResidual[NACCELSTATES];

// GPS Correction
double R_GPS[NGPSSTATES][NGPSSTATES];
double H_GPS[NGPSSTATES][NSTATES];
double K_GPS[NSTATES][NGPSSTATES];
double gpsResidual[NGPSSTATES];

double R_GPS2D[N2DGPSSTATES][N2DGPSSTATES];
double H_GPS2D[N2DGPSSTATES][NSTATES];
double K_GPS2D[NSTATES][N2DGPSSTATES];
double gpsResidual2D[N2DGPSSTATES];

// Barometer Correction
double R_BARO[NBAROSTATES][NBAROSTATES];
double H_BARO[NBAROSTATES][NSTATES];
double K_BARO[NSTATES][NBAROSTATES];
double baroRefAltitude;
double baroResidual[NBAROSTATES];

#ifdef FILTERTEST
double linNavStates[NSTATES];
double navStates[NSTATES];
double Pcorrection[NSTATES][NSTATES];
NavType NavStateError;
NavType linNavStateError;
#endif

void FsNavigation_setupNavigation(double *initialPosition, double initialHeading)
{
#ifdef NAVIGATION
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
    
    updateGravity();
    
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
            
            if (i<NACCELSTATES && j<NACCELSTATES) { R_ACCEL[i][j] = 0.0; }
            if (i<NACCELSTATES)                   { H_ACCEL[i][j] = 0.0; }
            if (j<NACCELSTATES)                   { K_ACCEL[i][j] = 0.0; }
            
            if (i<NGROUNDSTATES && j<NGROUNDSTATES) { R_GROUND[i][j] = 0.0; }
            if (i<NGROUNDSTATES)                    { H_GROUND[i][j] = 0.0; }
            if (j<NGROUNDSTATES)                    { K_GROUND[i][j] = 0.0; }
            
            if (i<NGPSSTATES && j<NGPSSTATES) { R_GPS[i][j] = 0.0; }
            if (i<NGPSSTATES)                 { H_GPS[i][j] = 0.0; }
            if (j<NGPSSTATES)                 { K_GPS[i][j] = 0.0; }
            
            if (i<NBAROSTATES && j<NBAROSTATES) { R_BARO[i][j] = 0.0; }
            if (i<NBAROSTATES)                  { H_BARO[i][j] = 0.0; }
            if (j<NBAROSTATES)                  { K_BARO[i][j] = 0.0; }
        }
    }
    
    H_ACCEL[ACCEL_ROLL][ROLL]   = 1.0;
    H_ACCEL[ACCEL_PITCH][PITCH] = 1.0;

    H_GPS[GPS_N][N]     = 1.0;
    H_GPS[GPS_E][E]     = 1.0;
    H_GPS[GPS_ALT][ALT] = 1.0;
    H_GPS[GPS_VN][VN]   = 1.0;
    H_GPS[GPS_VE][VE]   = 1.0;
    H_GPS[GPS_VD][VD]   = 1.0;
    
    H_GPS2D[GPS2D_N][N]   = 1.0;
    H_GPS2D[GPS2D_E][E]   = 1.0;
    H_GPS2D[GPS2D_VN][VN] = 1.0;
    H_GPS2D[GPS2D_VE][VE] = 1.0;
    
    H_BARO[BARO_ALT][ALT] = 1.0;
    
    // State Covariance (State Uncertainties)
    P[ROLL][ROLL]   = errorToVariance(5.0*degree2radian);
    P[PITCH][PITCH] = errorToVariance(5.0*degree2radian);
    P[YAW][YAW]     = errorToVariance(180.0*degree2radian);
    P[VN][VN]       = errorToVariance(1.0);
    P[VE][VE]       = errorToVariance(1.0);
    P[VD][VD]       = errorToVariance(1.0);
    P[N][N]         = errorToVariance(10.0);
    P[E][E]         = errorToVariance(10.0);
    P[ALT][ALT]     = errorToVariance(10.0);
    P[GBIAS_X][GBIAS_X] = errorToVariance(0.0*degree2radian);
    P[GBIAS_Y][GBIAS_Y] = errorToVariance(0.0*degree2radian);
    P[GBIAS_Z][GBIAS_Z] = errorToVariance(0.0*degree2radian);
    P[ABIAS_X][ABIAS_X] = errorToVariance(0.05*Gravity);
    P[ABIAS_Y][ABIAS_Y] = errorToVariance(0.05*Gravity);
    P[ABIAS_Z][ABIAS_Z] = errorToVariance(0.08*Gravity);
    P[GRAVITY][GRAVITY] = 0.0;//errorToVariance(0.1);

#ifdef FILTERTEST
    linNavStates[ROLL]  = NavData.eulerAngles[0];
    linNavStates[PITCH] = NavData.eulerAngles[1];
    linNavStates[YAW]   = NavData.eulerAngles[2];
    linNavStates[VN]    = NavData.velNED[0];
    linNavStates[VE]    = NavData.velNED[1];
    linNavStates[VD]    = NavData.velNED[2];
    linNavStates[N]     = NavData.position[0];
    linNavStates[E]     = NavData.position[1];
    linNavStates[ALT]   = NavData.position[2];
    
    navStates[ROLL]  = NavData.q_B_NED[0];
    navStates[PITCH] = NavData.q_B_NED[1];
    navStates[YAW]   = NavData.q_B_NED[2];
    navStates[VN]    = NavData.velNED[0];
    navStates[VE]    = NavData.velNED[1];
    navStates[VD]    = NavData.velNED[2];
    navStates[N]     = NavData.position[0];
    navStates[E]     = NavData.position[1];
    navStates[ALT]   = NavData.position[2];
#endif
#endif
}

void FsNavigation_performNavigation( double &navDt )
{
#ifdef NAVIGATION
    if (!navSetup) { return; }
    
    // Apply sensor updates
    if (NavData.state != Calibration && NavData.state != INS)
    {
        applyCorrections();
        NavData.updateCount[NavData.state]++;
        NavData.timestamp_diff[NavData.state] = NavData.sensorTimestamp[NavData.state] - NavData.timestamp;
        NavData.state = INS;
    }
    
    // Inertial Navigation
    if (NavData.state == INS)
    {
        NavData.imuTimestamp = nav_pIMUdata->timestamp;
        
        updateGravity();
        
        updateInputs(navDt);
        
        // Update Attitude
        performARHS(navDt);
        
        // Update full Navigation solution
        performINS(navDt);
        
        // Propogate Covariance Matrix
        propogateVariance(navDt);
   
        NavData.timestamp = getTime();
        
        NavData.updateCount[NavData.state]++;
        NavData.sensorTimestamp[NavData.state] = nav_pIMUdata->timestamp;
        NavData.timestamp_diff[NavData.state] = NavData.sensorTimestamp[NavData.state] - NavData.timestamp;

#ifdef SIMULATION
        // Update Navigation Truth
        updateTruth();
#endif
    }
    
    FsImu_zeroDelta();
#endif
}

void updateGravity()
{
    double hCenter;
    hCenter = RE + NavData.altitude_msl;
    NavData.gravity = GMe/(hCenter*hCenter);
}

void updateInputs( double &navDt )
{
    // Gravity
    double gravityNED[3];
    
    gravityNED[0] = 0.0;
    gravityNED[1] = 0.0;
    gravityNED[2] = NavData.gravity;
    
    FsNavigation_NEDToBody(NavData.gravityBody, gravityNED);
    
    // Inputs
    for (int i=0; i<3; i++)
    {
        // correct for gravity and bias
        NavData.stateInputs.dVelocity[i] = nav_pIMUdata->dVelocity[i] - NavData.accBias[i]*navDt + NavData.gravityBody[i]*navDt;
        NavData.stateInputs.dTheta[i]    = nav_pIMUdata->dTheta[i] - NavData.gyroBias[i]*navDt;
        
        //NavData.accelBody[i] = nav_pIMUdata->accel[i] - NavData.accBias[i] + NavData.gravityBody[i];
        //NavData.bodyRates[i] = nav_pIMUdata->gyro[i] - NavData.gyroBias[i]; //NavData.stateInputs.dTheta[i]/navDt;
        NavData.accelBody[i] = NavData.stateInputs.dVelocity[i]/navDt;
        NavData.bodyRates[i] = NavData.stateInputs.dTheta[i]/navDt;
    }
    
    NavData.accel_mag = sqrt(NavData.accelBody[0]*NavData.accelBody[0] + NavData.accelBody[1]*NavData.accelBody[1] + NavData.accelBody[2]*NavData.accelBody[2]);
}
void performARHS( double &navDt )
{    
    // Update quaternion using gyroscope
    gyroUpdate(navDt);
}

void gyroUpdate( double &navDt)
{
    // Variables
    double dThetaMag;
    double unitDir[3];
    double qOld[4];
    double dqGyro[4];
    
    // Store quaternion
    for (int i=0; i<4; i++)
    { qOld[i] = NavData.q_B_NED[i]; }

    // Angle change magnitude
    dThetaMag = sqrt( NavData.stateInputs.dTheta[0]*NavData.stateInputs.dTheta[0] +
                     NavData.stateInputs.dTheta[1]*NavData.stateInputs.dTheta[1] +
                     NavData.stateInputs.dTheta[2]*NavData.stateInputs.dTheta[2] );
    
    for (int i=0; i<3; i++)
    {
        if (dThetaMag > 0.0) { unitDir[i]  = NavData.stateInputs.dTheta[i] / dThetaMag; }
        else                 { unitDir[i] = 0.0; }
    }
    
    // Compute change in quaternion
    dqGyro[0] = cos(dThetaMag/2.0);
    dqGyro[1] = unitDir[0]*sin(dThetaMag/2.0);
    dqGyro[2] = unitDir[1]*sin(dThetaMag/2.0);
    dqGyro[3] = unitDir[2]*sin(dThetaMag/2.0);
    
    // Update quaternion
    quaternionProduct(NavData.q_B_NED, qOld, dqGyro);
    
    // Guarantee unit vector
    unitVector(NavData.q_B_NED, 4);
}

void performINS( double &navDt )
{
    double w_x_vel[3];
    
    // Euler Angles
    updateEulerAngles();
    
    // Body Velocity
    //accBody = (F - bodyRates x m*velBody) / mass
    crossProduct(w_x_vel, NavData.bodyRates, NavData.velBody);
    for (int i=0; i<3; i++)
    {
        // -bodyRates x velBody
        NavData.velBody[i] += NavData.stateInputs.dVelocity[i] - w_x_vel[i]*navDt;
    }
    
    // NED Velocity
    FsNavigation_bodyToNED(NavData.velNED, NavData.velBody);
    
    // N, E, Alt
    NavData.position[0] = NavData.position[0] + NavData.velNED[0]*navDt;
    NavData.position[1] = NavData.position[1] + NavData.velNED[1]*navDt;
    NavData.position[2] = NavData.position[2] - NavData.velNED[2]*navDt;
    
    // height_above_ellipsoid = MSL + geoidHeight
    NavData.altitude_msl = NavData.position[2] - NavData.geoidCorrection;
}

void propogateVariance( double &navDt )
{
    // Intermidiate variables
    double Qc[NSTATES][NSTATES] = {0.0};
    double Qc_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHI_Qc_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHItrans[NSTATES][NSTATES] = {0.0};
    double P_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHI_P_PHItrans[NSTATES][NSTATES] = {0.0};
 
    double sr    = sin(NavData.eulerAngles[0]);
    double cr    = cos(NavData.eulerAngles[0]);
    double sp    = sin(NavData.eulerAngles[1]);
    double cp    = cos(NavData.eulerAngles[1]);
    double sy    = sin(NavData.eulerAngles[2]);
    double cy    = cos(NavData.eulerAngles[2]);
    double tp    = tan(NavData.eulerAngles[1]);
    double secp  = 1.0/cos(NavData.eulerAngles[1]);
    double sec2p = secp*secp;
    double RH_h3 = (RE + NavData.position[2])*(RE + NavData.position[2])*(RE + NavData.position[2]);
    double RH_h4 = RH_h3*(RE + NavData.position[2]);

    // Roll
    PHI[ROLL][ROLL]    = 1.0 + cr*tp*NavData.stateInputs.dTheta[Y] - sr*tp*NavData.stateInputs.dTheta[Z];
    PHI[ROLL][PITCH]   = sr*sec2p*NavData.stateInputs.dTheta[Y] + cr*sec2p*NavData.stateInputs.dTheta[Z];
    PHI[ROLL][GBIAS_X] = -1.0*navDt;
    PHI[ROLL][GBIAS_Y] = (-sr*tp)*navDt;
    PHI[ROLL][GBIAS_Z] = (-cr*tp)*navDt;
    //Pitch
    PHI[PITCH][ROLL]   = -sr*NavData.stateInputs.dTheta[Y]- cr*NavData.stateInputs.dTheta[Z];
    PHI[PITCH][PITCH]  = 1.0;
    PHI[PITCH][GBIAS_Y] = -cr*navDt;
    PHI[PITCH][GBIAS_Z] = sr*navDt;
    // Yaw
    PHI[YAW][ROLL]    = cr*secp*NavData.stateInputs.dTheta[Y] - sr*secp*NavData.stateInputs.dTheta[Z];
    PHI[YAW][PITCH]   = secp*tp*sr*NavData.stateInputs.dTheta[Y] + secp*tp*cr*NavData.stateInputs.dTheta[Z];
    PHI[YAW][YAW]     = 1.0;
    PHI[YAW][GBIAS_Y] = -sr*secp*navDt;
    PHI[YAW][GBIAS_Z] = -cr*secp*navDt;
    
    // VN
    PHI[VN][ROLL]    = (cy*sp*cr+sr*sy)*NavData.stateInputs.dVelocity[Y] + (-cy*sp*sr+cr*sy)*NavData.stateInputs.dVelocity[Z];
    PHI[VN][PITCH]   = -sp*cy*NavData.stateInputs.dVelocity[X] + cy*cp*sr*NavData.stateInputs.dVelocity[Y] + cy*cp*cr*NavData.stateInputs.dVelocity[Z];
    PHI[VN][YAW]     = -cp*sy*NavData.stateInputs.dVelocity[X] + (-sy*sp*sr-cr*cy)*NavData.stateInputs.dVelocity[Y] + (-sy*sp*cr+sr*cy)*NavData.stateInputs.dVelocity[Z];
    PHI[VN][VN]      = 1.0;
    PHI[VN][ABIAS_X] = -cp*cy*navDt;
    PHI[VN][ABIAS_Y] = -(cy*sp*sr-cr*sy)*navDt;
    PHI[VN][ABIAS_Z] = -(cy*sp*cr+sr*sy)*navDt;
    
    // VE
    PHI[VE][ROLL]    = (-sr*cy+sp*cr*sy)*NavData.stateInputs.dVelocity[Y] + (-cr*cy-sp*sr*sy)*NavData.stateInputs.dVelocity[Z];
    PHI[VE][PITCH]   = -sp*sy*NavData.stateInputs.dVelocity[X] + cp*sr*sy*NavData.stateInputs.dVelocity[Y] + cp*cr*sy*NavData.stateInputs.dVelocity[Z];
    PHI[VE][YAW]     = cp*cy*NavData.stateInputs.dVelocity[X]+ + (-cr*sy+sp*sr*cy)*NavData.stateInputs.dVelocity[Y] + (sr*sy+sp*cr*sy)*NavData.stateInputs.dVelocity[Z];
    PHI[VE][VE]      = 1.0;
    PHI[VE][ABIAS_X] = -cp*sy*navDt;
    PHI[VE][ABIAS_Y] = -(cr*cy+sp*sr*sy)*navDt;
    PHI[VE][ABIAS_Z] = -(sr*cy+sp*cr*sy)*navDt;
    // VD
    PHI[VD][ROLL]    = cp*cr*NavData.stateInputs.dVelocity[Y] - cp*sr*NavData.stateInputs.dVelocity[Z];
    PHI[VD][PITCH]   = -cp*NavData.stateInputs.dVelocity[X] - sp*sr*NavData.stateInputs.dVelocity[Y] - sp*cr*NavData.stateInputs.dVelocity[Z];
    PHI[VD][VD]      = 1.0;
    PHI[VD][ABIAS_X] = sp*navDt;
    PHI[VD][ABIAS_Y] = -cp*sr*navDt;
    PHI[VD][ABIAS_Z] = -cp*cr*navDt;
    PHI[VD][GRAVITY] = 1.0*navDt;
    // N
    PHI[N][VN] = 1.0*navDt;
    PHI[N][N]  = 1.0;
    // E
    PHI[E][VE] = 1.0*navDt;
    PHI[E][E]  = 1.0;
    // ALT
    PHI[ALT][VD]  = -1.0*navDt;
    PHI[ALT][ALT] = 1.0;
    // BIAS
    PHI[GBIAS_X][GBIAS_X] = 1.0;
    PHI[GBIAS_Y][GBIAS_Y] = 1.0;
    PHI[GBIAS_Z][GBIAS_Z] = 1.0;
    PHI[ABIAS_X][ABIAS_X] = 1.0;
    PHI[ABIAS_Y][ABIAS_Y] = 1.0;
    PHI[ABIAS_Z][ABIAS_Z] = 1.0;
    // Gravity
    PHI[GRAVITY][VD]      = 2.0*GMe/RH_h3*navDt;
    PHI[GRAVITY][ALT]     = -6.0*GMe/RH_h4*NavData.velNED[2]*navDt;
    PHI[GRAVITY][GRAVITY] = 1.0;
    
    math_mtran(*PHItrans, *PHI, NSTATES, NSTATES);
    
    // Update Process Noise
    Qc[ROLL][ROLL]   = gyroCov[X][X];
    Qc[PITCH][PITCH] = gyroCov[Y][Y];
    Qc[YAW][YAW]     = gyroCov[Z][Z];
    Qc[VN][VN]       = accelCov[X][X];
    Qc[VE][VE]       = accelCov[Y][Y];
    Qc[VD][VD]       = accelCov[Z][Z];
    
    //Q = PHI*Qc*PHI^T dt;
    math_mmult(*Qc_PHItrans, *Qc, NSTATES, NSTATES, *PHItrans, NSTATES, NSTATES);
    math_mmult(*PHI_Qc_PHItrans, *PHI, NSTATES, NSTATES, *Qc_PHItrans, NSTATES, NSTATES);
    math_mgain(*Q, *PHI_Qc_PHItrans, navDt/2.0, NSTATES, NSTATES);
    
    // Propogate Covariance
    //P = PHI*P*PHI^T + Q;
    math_mmult(*P_PHItrans, *P, NSTATES, NSTATES, *PHItrans, NSTATES, NSTATES);
    math_mmult(*PHI_P_PHItrans, *PHI, NSTATES, NSTATES, *P_PHItrans, NSTATES, NSTATES);
    math_madd(*P, *PHI_P_PHItrans, *Q, NSTATES, NSTATES);
}

void applyCorrections()
{
    if (NavData.state == BaroUpdate)
    {
        filterUpdate(baroResidual, *R_BARO, *H_BARO, *K_BARO, NBAROSTATES);
    }
    else if (NavData.state == GPSUpdate)
    {
        filterUpdate(gpsResidual, *R_GPS, *H_GPS, *K_GPS, NGPSSTATES);
    }
    else if (NavData.state == GPSUpdate2D)
    {
        filterUpdate(gpsResidual2D, *R_GPS2D, *H_GPS2D, *K_GPS2D, N2DGPSSTATES);
    }
    else if (NavData.state == AccelUpdate)
    {
        filterUpdate(accelResidual, *R_ACCEL, *H_ACCEL, *K_ACCEL, NACCELSTATES);
    }
    else if (NavData.state == GroundAlign)
    { }
    
    NavData.velNED[0] += stateError[VN];
    NavData.velNED[1] += stateError[VE];
    NavData.velNED[2] += stateError[VD];
    FsNavigation_NEDToBody(NavData.velBody, NavData.velNED);
    
    NavData.position[0] += stateError[N];
    NavData.position[1] += stateError[E];
    NavData.position[2] += stateError[ALT];

    NavData.eulerAngles[0] += stateError[ROLL];
    NavData.eulerAngles[1] += stateError[PITCH];
    NavData.eulerAngles[2] += stateError[YAW];
    updateQuaternions();
    
    NavData.gyroBias[0] += stateError[GBIAS_X];
    NavData.gyroBias[1] += stateError[GBIAS_Y];
    NavData.gyroBias[2] += stateError[GBIAS_Z];
    
    NavData.accBias[0] += stateError[ABIAS_X];
    NavData.accBias[1] += stateError[ABIAS_Y];
    NavData.accBias[2] += stateError[ABIAS_Z];
    
    NavData.gravity += stateError[GRAVITY];
    
    for (int i = 0; i < NSTATES; i++)
    {
        stateError[i] = 0.0;
    }
}

void filterUpdate(double* residual, double* R, double* H, double* K, int nMeas)
{
    double HT[NSTATES][nMeas];
    double P_HT[NSTATES][nMeas];
    double H_P_HT[nMeas][nMeas];
    double H_P_HT_R[nMeas][nMeas];
    double inv_H_P_HT_R[nMeas][nMeas];
    double HT_inv_H_P_HT_R[NSTATES][nMeas];
    
    double K_H[NSTATES][NSTATES];
    double diagOnes_K_H[NSTATES][NSTATES];
    double Pprev[NSTATES][NSTATES];
    
    for (int i = 0; i < NSTATES; i++)
    {
        for (int j = 0; j < NSTATES; j++)
        { Pprev[i][j] = P[i][j]; }
    }
    
    // NSTATES x NMESAUREMENTS
    // 10x6   = (10x10) * (10x6)  *    (  6x10  * 10x10 *  10x6   +   6x6 )
    // K =    P    * H' * inv( H *   P   * H' + R);
    math_mtran(*HT, H, NSTATES, nMeas);
    math_mmult(*P_HT, *P, NSTATES, NSTATES, *HT, NSTATES, nMeas);
    math_mmult(*H_P_HT, H, nMeas, NSTATES, *P_HT, NSTATES, nMeas);
    math_madd(*H_P_HT_R, *H_P_HT, R, nMeas, nMeas);
    math_minv(*inv_H_P_HT_R, *H_P_HT_R, nMeas);
    math_mmult(*HT_inv_H_P_HT_R, *HT, NSTATES, nMeas, *inv_H_P_HT_R, nMeas, nMeas);
    math_mmult(K, *P, NSTATES, NSTATES, *HT_inv_H_P_HT_R, NSTATES, nMeas);

    //    10x1    =  10x6  *   6x1
    // stateError = K * Residual
    math_mmult(stateError, K, residual, NSTATES, nMeas);
    
    //10x10 = (10x10 -  10x6  *  6x10 ) * 10x10
    // P    =  (  I  - K * H) *   P
    math_mmult(*K_H, K, NSTATES, nMeas, H, nMeas, NSTATES);
    math_msubtract(*diagOnes_K_H, *diagOnes, *K_H, NSTATES, NSTATES);
    math_mmult(*P, *diagOnes_K_H, NSTATES, NSTATES, *Pprev, NSTATES, NSTATES);
    
    for (int i = 0; i < NSTATES; i++)
    {
        for (int j = 0; j < NSTATES; j++)
        {
            Pcorrection[i][j] = diagOnes_K_H[i][j];
        }
    }
}

void FsNavigation_performAccelerometerUpdate()
{
    if (!navSetup) { return; }
    
    if(NavData.state != Calibration && NavData.state != INS)
    {
        NavData.skippedUpdateCount[AccelUpdate]++;
        return;
    }
    
    static int stable_count = 0;
    static bool variance_set = false;
    double accBias_error[3];
    double M, M2, Myz, Myz2, M2Myz;
    double accel[3];
    double droll_dbias[3];
    double dpitch_dbias[3];
    double roll_error;
    double pitch_error;
    double accel_error;
    
    for (int i=0; i<3; i++)
    {
        accel[i] = nav_pIMUdata->accel[i];
    }
    
    Myz = sqrt(accel[1]*accel[1] + accel[2]*accel[2]);
    
    NavData.accel_roll  = -atan2(accel[1], -accel[2]);
    NavData.accel_pitch = atan2(accel[0], Myz);
    
    // What is the effect of acceleration bias uncertainty on pitch and roll?
    // Linearization point is at current accleration
    // Change in acceleration is zero, change in bias is uncertainty
    if (!variance_set)
    {
        M  = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        M2 = M*M; Myz2  = Myz*Myz; M2Myz = M2*Myz;
    
        droll_dbias[0] = 0.0;
        droll_dbias[1] = -accel[2]/(Myz2);
        droll_dbias[2] = accel[1]/(Myz);
    
        dpitch_dbias[0] = -Myz/(M2);
        dpitch_dbias[1] = accel[0]*accel[1]/(M2Myz);
        dpitch_dbias[2] = accel[0]*accel[2]/(M2Myz);

        accBias_error[0] = 3.0*sqrt(accelCov[0][0]) + 3.0*sqrt(P[ABIAS_X][ABIAS_X]);
        accBias_error[1] = 3.0*sqrt(accelCov[1][1]) + 3.0*sqrt(P[ABIAS_Y][ABIAS_Y]);
        accBias_error[2] = 3.0*sqrt(accelCov[2][2]) + 3.0*sqrt(P[ABIAS_Z][ABIAS_Z]);
    
        accel_error = sqrt(accBias_error[0]*accBias_error[0] + accBias_error[1]*accBias_error[1] + accBias_error[2]*accBias_error[2]);
        roll_error  = droll_dbias[0]*accBias_error[0]  + droll_dbias[1]*accBias_error[1]  + droll_dbias[2]*accBias_error[2];
        pitch_error = dpitch_dbias[0]*accBias_error[0] + dpitch_dbias[1]*accBias_error[1] + dpitch_dbias[2]*accBias_error[2];

        //std::cout << roll_error * radian2degree << " " << pitch_error * radian2degree << std::endl;
    
        R_ACCEL[ACCEL_ROLL][ACCEL_ROLL]   = errorToVariance(roll_error);
        R_ACCEL[ACCEL_PITCH][ACCEL_PITCH] = errorToVariance(pitch_error);
    
        if (NavData.state != Calibration) { variance_set = true; }
    }
    
    if (NavData.accel_mag < 0.4
        && !nav_pIMUdata->highDynamics )
    {
        stable_count++;
        if (stable_count > 10 && NavData.state != Calibration)
        {
            accelResidual[ACCEL_ROLL]  = NavData.accel_roll  - NavData.eulerAngles[0];
            accelResidual[ACCEL_PITCH] = NavData.accel_pitch - NavData.eulerAngles[1];
            
            NavData.state = AccelUpdate;
            NavData.sensorTimestamp[NavData.state] = getTime();
        }
    }
    else
    {
        stable_count = 0;
    }
}

void FsNavigation_performGPSUpdate(GpsType* gpsData)
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }

    // MSL correction
    if (gpsData->positionValid)
    {
        // height_above_ellipsoid = MSL + geoidHeight
        NavData.geoidCorrection = gpsData->posENU[2] - gpsData->alt_msl;
    }
    
    if (gpsData->positionValid && gpsData->velocityValid)
    {
        if(NavData.state != INS)
        {
            if (gpsData->gpsFix == FIX2D) { NavData.skippedUpdateCount[GPSUpdate2D]++; }
            else { NavData.skippedUpdateCount[GPSUpdate]++; }

            return;
        }
        
        double horizPosVar = errorToVariance(gpsData->horizPosAcc);
        double vertPosVar  = errorToVariance(gpsData->vertPosAcc);
        double speedVar    = errorToVariance(gpsData->speedAcc);
        
        if (gpsData->gpsFix == FIX2D)
        {
            R_GPS2D[GPS2D_N][GPS2D_N]   = horizPosVar;
            R_GPS2D[GPS2D_E][GPS2D_E]   = horizPosVar;
            R_GPS2D[GPS2D_VN][GPS2D_VN] = speedVar;
            R_GPS2D[GPS2D_VE][GPS2D_VE] = speedVar;
            
            gpsResidual2D[GPS2D_N]  = gpsData->posENU[1] - NavData.position[0];
            gpsResidual2D[GPS2D_E]  = gpsData->posENU[0] - NavData.position[1];
            gpsResidual2D[GPS2D_VN] = gpsData->velNED[0] - NavData.velNED[0];
            gpsResidual2D[GPS2D_VE] = gpsData->velNED[1] - NavData.velNED[1];
            
            NavData.state = GPSUpdate2D;
        }
        else //(gpsData->gpsFix == FIX3D)
        {
            R_GPS[GPS_N][GPS_N]     = horizPosVar;
            R_GPS[GPS_E][GPS_E]     = horizPosVar;
            R_GPS[GPS_ALT][GPS_ALT] = vertPosVar;
            R_GPS[GPS_VN][GPS_VN]   = speedVar;
            R_GPS[GPS_VE][GPS_VE]   = speedVar;
            R_GPS[GPS_VD][GPS_VD]   = speedVar;
            
            gpsResidual[GPS_N]   = gpsData->posENU[1] - NavData.position[0];
            gpsResidual[GPS_E]   = gpsData->posENU[0] - NavData.position[1];
            gpsResidual[GPS_ALT] = gpsData->posENU[2] - NavData.position[2];
            gpsResidual[GPS_VN]  = gpsData->velNED[0] - NavData.velNED[0];
            gpsResidual[GPS_VE]  = gpsData->velNED[1] - NavData.velNED[1];
            gpsResidual[GPS_VD]  = gpsData->velNED[2] - NavData.velNED[2];
            
            NavData.state = GPSUpdate;
        }
        FsGPS_resetPositionValid();
        FsGPS_resetVelocityValid();
        NavData.sensorTimestamp[NavData.state] = gpsData->timestamp;
    }
}

void FsNavigation_performBarometerUpdate(BarometerType* baroData)
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    if(NavData.state != INS)
    {
        NavData.skippedUpdateCount[BaroUpdate]++;
        return;
    }
    
    R_BARO[BARO_ALT][BARO_ALT] = FsBarometer_getAltitudeVariance();
    baroResidual[BARO_ALT]     = baroData->altitude + baroRefAltitude - NavData.position[2];

    NavData.state = BaroUpdate;
    NavData.sensorTimestamp[NavData.state] = baroData->timestamp;
}

void FsNavigation_groundAlign()
{
    if ( (NavData.state == Calibration) || (!navSetup) ) { return; }
    
    if(NavData.state != INS)
    {
        NavData.skippedUpdateCount[GroundAlign]++;
        return;
    }

    NavData.state = GroundAlign;
    NavData.sensorTimestamp[NavData.state] = getTime();
}

void FsNavigation_calibrateIMU()
{
    static bool firstTime = true;
    
    if (imuCalibrated) { return; }
    double tempAcc;
    double tempGyro;
    static double gravity_body[3];
    static double g_error_est[3];
    double roll_error, pitch_error;
    double sr, cr, sp, cp;
    
    if (firstTime)
    {
        for (int i=0; i<3; i++)
        {
            gyroError[i]  = new SensorErrorType;
            accelError[i] = new SensorErrorType;
        }
        sr = sin(NavData.accel_roll);
        cr = cos(NavData.accel_roll);
        sp = sin(NavData.accel_pitch);
        cp = cos(NavData.accel_pitch);
        
        gravity_body[0] = -sp * Gravity;
        gravity_body[1] = cp*sr * Gravity;
        gravity_body[2] = cp*cr * Gravity;
        
        roll_error  = varianceToError(R_ACCEL[ACCEL_ROLL][ACCEL_ROLL]);
        pitch_error = varianceToError(R_ACCEL[ACCEL_PITCH][ACCEL_PITCH]);
        g_error_est[0] = Gravity*(-cp*pitch_error);
        g_error_est[1] = Gravity*(-sp*sr*pitch_error + cp*cr*roll_error);
        g_error_est[2] = Gravity*(-sp*cr*pitch_error - cp*sr*roll_error);
        
        firstTime = false;
    }
    
    // Get Sum
    for (int i=0; i<3; i++)
    {
        // Gyroscope
        tempGyro = nav_pIMUdata->gyro[i];
        gyroError[i]->sum    += tempGyro;
        gyroError[i]->sumSqr += tempGyro * tempGyro;
        if (nav_pIMUdata->gyro[i] > gyroError[i]->max) { gyroError[i]->max =  tempGyro; }
        if (nav_pIMUdata->gyro[i] < gyroError[i]->min) { gyroError[i]->min =  tempGyro; }
        
        // Accelerometer
        tempAcc = nav_pIMUdata->accel[i] + gravity_body[i];
        
        accelError[i]->sum    += tempAcc;
        accelError[i]->sumSqr += tempAcc * tempAcc;
        if (tempAcc > accelError[i]->max) { accelError[i]->max =  tempAcc; }
        if (tempAcc < accelError[i]->min) { accelError[i]->min =  tempAcc; }
    }
    
    NavData.updateCount[Calibration]++;
    NavData.sensorTimestamp[Calibration] = getTime();
    
    // Compute statistics
    if (NavData.updateCount[Calibration] >= maxCalCounter)
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

        double gyroQuatizationVariance =  errorToVariance(nav_pIMUdata->gyroQuantizationError_dps*degree2radian);
        double accelQuatizationVariance = errorToVariance(nav_pIMUdata->accelQuantizationError_g*Gravity);
        
        gyroCov[0][0]  = gyroError[0]->variance + gyroQuatizationVariance;
        gyroCov[1][1]  = gyroError[1]->variance + gyroQuatizationVariance;
        gyroCov[2][2]  = gyroError[2]->variance + gyroQuatizationVariance;
        accelCov[0][0] = accelError[0]->variance + accelQuatizationVariance;
        accelCov[1][1] = accelError[1]->variance + accelQuatizationVariance;
        accelCov[2][2] = accelError[2]->variance + accelQuatizationVariance;
        
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
        
        NavData.eulerAngles[0] = NavData.accel_roll;
        NavData.eulerAngles[1] = NavData.accel_pitch;
        updateQuaternions();
      
        P[ROLL][ROLL]   = R_ACCEL[ACCEL_ROLL][ACCEL_ROLL];
        P[PITCH][PITCH] = R_ACCEL[ACCEL_PITCH][ACCEL_PITCH];
        
        P[ABIAS_X][ABIAS_X] = errorToVariance(g_error_est[0]);
        P[ABIAS_Y][ABIAS_Y] = errorToVariance(g_error_est[1]);
        P[ABIAS_Z][ABIAS_Z] = errorToVariance(g_error_est[2]);
        
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

double FsNavigation_getNavAlt()
{
    return NavData.position[2];
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

double* FsNavigation_getStateError()
{
    return stateError;
}

double* FsNavigation_getCovarianceCorrection()
{
    return *Pcorrection;
}

double* FsNavigation_getBaroKalmanGain()
{
    return *K_BARO;
}

double* FsNavigation_getBaroResidual()
{
    return baroResidual;
}

double* FsNavigation_getGPSMeasVariance()
{
    return *R_GPS;
}

double* FsNavigation_getGPSResidual()
{
    return gpsResidual;
}
double* FsNavigation_getAccelMeasVariance()
{
    return *R_ACCEL;
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
        truthNavData.position[0] = pDyn->getPosNED()[0];
        truthNavData.position[1] = pDyn->getPosNED()[1];
        truthNavData.position[2] = -pDyn->getPosNED()[2];
        
        truthNavData.velBody[0] = pDyn->getVelBody()[0];
        truthNavData.velBody[1] = pDyn->getVelBody()[1];
        truthNavData.velBody[2] = pDyn->getVelBody()[2];
        
        truthNavData.velNED[0] = pDyn->getVelNED()[0];
        truthNavData.velNED[1] = pDyn->getVelNED()[1];
        truthNavData.velNED[2] = pDyn->getVelNED()[2];

        truthNavData.eulerAngles[0] = pDyn->getEulerAngles()[0];
        truthNavData.eulerAngles[1] = pDyn->getEulerAngles()[1];
        truthNavData.eulerAngles[2] = pDyn->getEulerAngles()[2];
        
        truthNavData.q_B_NED[0] = pDyn->get_q_B_NED()[0];
        truthNavData.q_B_NED[1] = pDyn->get_q_B_NED()[1];
        truthNavData.q_B_NED[2] = pDyn->get_q_B_NED()[2];
        truthNavData.q_B_NED[3] = pDyn->get_q_B_NED()[3];
        
        truthNavData.accelBody[0] = pDyn->getAccBody()[0];
        truthNavData.accelBody[1] = pDyn->getAccBody()[1];
        truthNavData.accelBody[2] = pDyn->getAccBody()[2];
        
        truthNavData.bodyRates[0] = pDyn->getBodyRates()[0];
        truthNavData.bodyRates[1] = pDyn->getBodyRates()[1];
        truthNavData.bodyRates[2] = pDyn->getBodyRates()[2];
        
        truthNavData.imuTimestamp = pDyn->getTimestamp();
    }
    
    if (pAtmo)
    {
        truthNavData.accel_roll  = pAtmo->getGravityRoll();
        truthNavData.accel_pitch = pAtmo->getGravityPitch();
    }
    
    for (int i=0; i<3; i++)
    {
        NavError.position[i]    = NavData.position[i]     - truthNavData.position[i];
        NavError.velNED[i]      = NavData.velNED[i]       - truthNavData.velNED[i];
        NavError.velBody[i]     = NavData.velBody[i]      - truthNavData.velBody[i];
        NavError.eulerAngles[i] = (NavData.eulerAngles[i] - truthNavData.eulerAngles[i]) * radian2degree;
        NavError.q_B_NED[i]     = NavData.q_B_NED[i]      - truthNavData.q_B_NED[i];
        NavError.accelBody[i]   = NavData.accelBody[i]    - truthNavData.accelBody[i];
        NavError.bodyRates[i]   = (NavData.bodyRates[i]   - truthNavData.bodyRates[i]) * radian2degree;
        
        if (NavError.eulerAngles[i] > 180.0) { NavError.eulerAngles[i] -= 360.0; }
        else if (NavError.eulerAngles[i] < -180.0) { NavError.eulerAngles[i] += 360.0; }
    }
    NavError.q_B_NED[3] = truthNavData.q_B_NED[3] - NavData.q_B_NED[3];
    
    NavError.accel_roll  = (truthNavData.accel_roll  - NavData.accel_roll ) * radian2degree;
    NavError.accel_pitch = (truthNavData.accel_pitch - NavData.accel_pitch) * radian2degree;
    
    NavError.imuTimestamp = (NavData.imuTimestamp - truthNavData.imuTimestamp)*1000.0;
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

inline void updateQuaternions()
{
    double cr2;
    double cp2;
    double cy2;
    double sr2;
    double sp2;
    double sy2;
    
    cr2 = cos(NavData.eulerAngles[0]/2.0);
    cp2 = cos(NavData.eulerAngles[1]/2.0);
    cy2 = cos(NavData.eulerAngles[2]/2.0);
        
    sr2 = sin(NavData.eulerAngles[0]/2.0);
    sp2 = sin(NavData.eulerAngles[1]/2.0);
    sy2 = sin(NavData.eulerAngles[2]/2.0);
        
    NavData.q_B_NED[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
    NavData.q_B_NED[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
    NavData.q_B_NED[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
    NavData.q_B_NED[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
    
    unitVector(NavData.q_B_NED, 4);
}

inline void unitVector(double* vector, int n)
{
    // Magnitude
    double mag = 0.0;
    for (int i = 0; i < n; i++)
    {
        mag += vector[i]*vector[i];
    }
    mag = sqrt(mag);
    if (mag == 0.0) return;
    
    // Unit vector
    for (int i = 0; i < n; i++)
    {
        vector[i] = vector[i]/mag;
    }
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
            
            for(int k = 0; k < ncols1; k++)
            {
                Arow = i; Acol = k; Brow = k; Bcol = j;
                Aik = *(A+Arow*ncols1+Acol);     // A[i][k]
                Bkj = *(B+Brow*ncols2+Bcol);     // B[k][j]
                *(result+i*ncols2+j) += Aik*Bkj; //C[i][j] = A[i][k]*B[k][j]
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
        U[i][i] = 1.0; // U diagonals = 1
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
            sum = 0.0;
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

inline void symmetric(double* matrix, int nrow, int ncol)
{
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            *(matrix + j*ncol + i) = *(matrix + i*ncol + j);// matrix[i][j]
        }
    }
}
