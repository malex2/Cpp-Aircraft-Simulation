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
    #include "imu_model.hpp"
    #include "atmosphere_model.hpp"
#endif

// Data
NavType NavData;
const IMUtype* nav_pIMUdata = 0;
#ifdef SIMULATION
    NavType truthNavData;
    NavType NavError;
#endif

// Booleans
bool navSetup = false;
bool useTruthStateTransitionMatrix = false;
bool useTruthGravity = false;
bool nav_apply_velocity_corrections = false;
bool use_truth_bias = false;
bool use_truth_attitude = false;

// Sensor Errors/Calibration
SensorErrorType calAccelError[3];
SensorErrorType calGyroError[3];

int maxCalCounter;

double gyroCov[3][3];
double accelCov[3][3];

double imuToNavRateRatio;

// NED delta velocity and delta position errors due to integration errors under attitude rates
const double ATT_ROT_VAR    = errorToVariance(0.1*degree2radian);
const double VEL_ROT_VAR_NE = errorToVariance(0.05);
const double VEL_ROT_VAR_D  = errorToVariance(0.05);
const double POS_ROT_VAR    = errorToVariance(0.1);
const double ROT_ERR_RATE_THRESH = 10.0 * degree2radian;
double vRotComp[3];

bool groundFlag;
bool movementFlag;

// Simulation Models
#ifdef SIMULATION
    class DynamicsModel*   pDyn = 0;
    class IMUModelBase*    pImu = 0;
    class AtmosphereModel* pAtmo = 0;
#endif

// KALMAN FILTER
double PHI[NSTATES][NSTATES];
double P[NSTATES][NSTATES];
double Q[NSTATES][NSTATES];
double stateError[NSTATES];
double stateErrorLatch[NSTATES];
double stateErrorAccum[NSTATES];
double diagOnes[NSTATES][NSTATES];

#ifdef SIMULATION
bool observabilityTestStates[NSTATES];
#endif

// Ground Align Correction
double R_GROUND[NGROUNDSTATES][NGROUNDSTATES];
double H_GROUND[NGROUNDSTATES][NSTATES];
double K_GROUND[NSTATES][NGROUNDSTATES];
double groundResidual[NGROUNDSTATES];
#ifdef SIMULATION
    ObservabilityTestType GROUND_Observability(NGROUNDSTATES, *H_GROUND, observabilityTestStates);
#endif

double position_ground_latch[3];
double yaw_ground_latch;
double yaw_var_ground_latch;

// Accelerometer Correction
double R_ACCEL[NACCELSTATES][NACCELSTATES];
double H_ACCEL[NACCELSTATES][NSTATES];
double K_ACCEL[NSTATES][NACCELSTATES];
double accelResidual[NACCELSTATES];
#ifdef SIMULATION
    ObservabilityTestType ACCEL_Observability(NACCELSTATES, *H_ACCEL, observabilityTestStates);
#endif

// GPS Correction
double R_GPS[NGPSSTATES][NGPSSTATES];
double H_GPS[NGPSSTATES][NSTATES];
double K_GPS[NSTATES][NGPSSTATES];
double gpsResidual[NGPSSTATES];
#ifdef SIMULATION
    ObservabilityTestType GPS_Observability(NGPSSTATES, *H_GPS, observabilityTestStates);
#endif

double R_GPS2D[N2DGPSSTATES][N2DGPSSTATES];
double H_GPS2D[N2DGPSSTATES][NSTATES];
double K_GPS2D[NSTATES][N2DGPSSTATES];
double gpsResidual2D[N2DGPSSTATES];
#ifdef SIMULATION
    ObservabilityTestType GPS2D_Observability(N2DGPSSTATES, *H_GPS2D, observabilityTestStates);
#endif

// Barometer Correction
double R_BARO[NBAROSTATES][NBAROSTATES];
double H_BARO[NBAROSTATES][NSTATES];
double K_BARO[NSTATES][NBAROSTATES];
double baroRefAltitude;
double baroResidual[NBAROSTATES];
#ifdef SIMULATION
    ObservabilityTestType BARO_Observability(NBAROSTATES, *H_BARO, observabilityTestStates);
#endif

#ifdef FILTERTEST
double linNavStates[NSTATES];
double navStates[NSTATES];
double Pcorrection[NSTATES][NSTATES];
NavType NavStateError;
NavType linNavStateError;
#endif

void FsNavigation_setupNavigation(double *initialPosition, double initialHeading, bool loadIMUCalibration)
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
    
    NavData.state = Nav_Startup;
    NavData.allowLoadIMUCal = loadIMUCalibration;
    
    groundFlag = true;
    movementFlag = false;
    
    // Variables
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            gyroCov[i][j]  = 0.0;
            accelCov[i][j] = 0.0;
        }
        position_ground_latch[i] = NavData.position[i];
        vRotComp[i] = 0.0;
    }
    yaw_ground_latch = NavData.eulerAngles[2];
    imuToNavRateRatio = 4.0;
    
    // Calibration
    maxCalCounter = 100;
    
    // KALMAN FILTER
    for (int i=0; i<NSTATES; i++)
    {
        stateError[i] = 0.0;
        stateErrorLatch[i] = 0.0;
        stateErrorAccum[i] = 0.0;
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
    
    H_GROUND[GROUND_N][N]       = 1.0;
    H_GROUND[GROUND_E][E]       = 1.0;
    H_GROUND[GROUND_ALT][ALT]   = 1.0;
    H_GROUND[GROUND_VN][VN]     = 1.0;
    H_GROUND[GROUND_VE][VE]     = 1.0;
    H_GROUND[GROUND_VD][VD]     = 1.0;
    H_GROUND[GROUND_YAW][ATT_Z] = 1.0;
    
    H_ACCEL[ACCEL_ROLL][ATT_X]  = 1.0;
    H_ACCEL[ACCEL_PITCH][ATT_Y] = 1.0;

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
    P[ATT_X][ATT_X] = errorToVariance(5.0*degree2radian);
    P[ATT_Y][ATT_Y] = errorToVariance(5.0*degree2radian);
    P[ATT_Z][ATT_Z] = errorToVariance(180.0*degree2radian);
    P[VN][VN]       = errorToVariance(0.0);
    P[VE][VE]       = errorToVariance(0.0);
    P[VD][VD]       = errorToVariance(0.0);
    P[N][N]         = errorToVariance(0.0);
    P[E][E]         = errorToVariance(0.0);
    P[ALT][ALT]     = errorToVariance(0.0);
    P[GBIAS_X][GBIAS_X] = errorToVariance(3.0*0.08*degree2radian);
    P[GBIAS_Y][GBIAS_Y] = errorToVariance(3.0*0.08*degree2radian);
    P[GBIAS_Z][GBIAS_Z] = errorToVariance(3.0*0.08*degree2radian);
    P[ABIAS_X][ABIAS_X] = errorToVariance(3.0*0.05*Gravity);
    P[ABIAS_Y][ABIAS_Y] = errorToVariance(3.0*0.05*Gravity);
    P[ABIAS_Z][ABIAS_Z] = errorToVariance(3.0*0.08*Gravity);
    P[GRAVITY][GRAVITY] = errorToVariance(0.01*Gravity);

    yaw_var_ground_latch = P[ATT_Z][ATT_Z];
#ifdef FILTERTEST
    linNavStates[ATT_X] = NavData.eulerAngles[0];
    linNavStates[ATT_Y] = NavData.eulerAngles[1];
    linNavStates[ATT_Z] = NavData.eulerAngles[2];
    linNavStates[VN]    = NavData.velNED[0];
    linNavStates[VE]    = NavData.velNED[1];
    linNavStates[VD]    = NavData.velNED[2];
    linNavStates[N]     = NavData.position[0];
    linNavStates[E]     = NavData.position[1];
    linNavStates[ALT]   = NavData.position[2];
    
    navStates[ATT_X] = NavData.q_B_NED[0];
    navStates[ATT_Y] = NavData.q_B_NED[1];
    navStates[ATT_Z] = NavData.q_B_NED[2];
    navStates[VN]    = NavData.velNED[0];
    navStates[VE]    = NavData.velNED[1];
    navStates[VD]    = NavData.velNED[2];
    navStates[N]     = NavData.position[0];
    navStates[E]     = NavData.position[1];
    navStates[ALT]   = NavData.position[2];
#endif
#ifdef SIMULATION
    observabilityTestStates[ATT_X]   = true;
    observabilityTestStates[ATT_Y]   = true;
    observabilityTestStates[ATT_Z]   = true;
    observabilityTestStates[VN]      = true;
    observabilityTestStates[VE]      = true;
    observabilityTestStates[VD]      = true;
    observabilityTestStates[N]       = true;
    observabilityTestStates[E]       = true;
    observabilityTestStates[ALT]     = true;
    observabilityTestStates[GBIAS_X] = true;
    observabilityTestStates[GBIAS_Y] = true;
    observabilityTestStates[GBIAS_Z] = true;
    observabilityTestStates[ABIAS_X] = true;
    observabilityTestStates[ABIAS_Y] = true;
    observabilityTestStates[ABIAS_Z] = true;
    observabilityTestStates[GRAVITY] = true;
    
    
    NavError.gravity = 0.0;
#endif
    
    navSetup = true;
#endif
}

void FsNavigation_performNavigation( double &navDt )
{
#ifdef NAVIGATION
    if (!navSetup || !nav_pIMUdata || !nav_pIMUdata->IMUgood) { return; }
    
    // Apply sensor updates
    if (NavData.state != Nav_Startup && NavData.state != INS && NavData.state != Calibration)
    {
        applyCorrections();
        NavData.updateCount[NavData.state]++;
        NavData.timestamp_diff[NavData.state] = NavData.sensorTimestamp[NavData.state] - NavData.timestamp;
        
        if (NavData.updateCount[Calibration] < maxCalCounter)
        {
            NavData.state = Calibration;
        }
        else
        {
            NavData.state = INS;
        }
    }
    
    // Inertial Navigation
    if (NavData.state == INS)
    {
#ifdef SIMULATION
        // Update Navigation Truth
        updateTruth();
#endif
        NavData.imuTimestamp = nav_pIMUdata->timestamp;
        
        updateGravity();
        
        updateInputs(navDt);
        
        // Update Attitude
        performARHS(navDt);
        
        // Apply gravity (must be done after attitude update) and compute magnitudes
        preINSCalculations(navDt);
        
        // Update full Navigation solution
        performINS(navDt);
        
        // Propogate Covariance Matrix
        propogateVariance(navDt);
   
        NavData.timestamp = getTime();
        
        NavData.updateCount[NavData.state]++;
        NavData.sensorTimestamp[NavData.state] = nav_pIMUdata->timestamp;
        NavData.timestamp_diff[NavData.state] = NavData.sensorTimestamp[NavData.state] - NavData.timestamp;

#ifdef SIMULATION
        // Update truth errors
        computeTruthErrors();
        
        // Observability
        //copmuteObservability(GPS_Observability);
#endif
    }
#endif
}

void updateGravity()
{
    double hCenter;
    hCenter = RE + NavData.altitude_msl;
    NavData.gravity = GMe/(hCenter*hCenter) + NavData.gravityBias;
    if (useTruthGravity)
    {
        NavData.gravity = truthNavData.gravity;
    }
}

void updateInputs( double &navDt )
{
    if (nav_apply_velocity_corrections)
    {
        crossProduct(vRotComp, nav_pIMUdata->dTheta, nav_pIMUdata->dVelocity);
    }
    
    // Inputs
    for (int i=0; i<3; i++)
    {
        NavData.velRotationComp[i] = 0.5 * vRotComp[i];

        // correct for bias and integration errors
        NavData.stateInputs.dVelocity[i] = nav_pIMUdata->dVelocity[i] - NavData.accBias[i]*navDt + nav_pIMUdata->scullingCorrection[i] + NavData.velRotationComp[i];
        NavData.stateInputs.dTheta[i]    = nav_pIMUdata->dTheta[i] - NavData.gyroBias[i]*navDt + nav_pIMUdata->coningCorrection[i];
    }
    
    // IMU delta velocity computed in previous body frame. This rotation must be done before attitude update
    FsNavigation_bodyToNED(NavData.deltaVelNED, NavData.stateInputs.dVelocity);
}
void performARHS( double &navDt )
{    
    // Update quaternion using gyroscope
    gyroUpdate(navDt);
    
    // Euler Angles
    updateEulerAngles();
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

void preINSCalculations( double &navDt )
{
    // Apply Gravity
    double gravityNED[3];
    gravityNED[0] = 0.0;
    gravityNED[1] = 0.0;
    gravityNED[2] = NavData.gravity;
    FsNavigation_NEDToBody(NavData.gravityBody, gravityNED);
    
    for (int i = 0; i < 3; i++)
    {
        NavData.stateInputs.dVelocity[i] += NavData.gravityBody[i]*navDt;
        NavData.accelBody[i] = NavData.stateInputs.dVelocity[i]/navDt;
        NavData.bodyRates[i] = NavData.stateInputs.dTheta[i]/navDt;
    }
    NavData.deltaVelNED[2] += NavData.gravity*navDt;
    
    NavData.accel_mag = sqrt(NavData.accelBody[0]*NavData.accelBody[0] + NavData.accelBody[1]*NavData.accelBody[1] + NavData.accelBody[2]*NavData.accelBody[2]);
    NavData.rates_mag = sqrt(NavData.bodyRates[0]*NavData.bodyRates[0] + NavData.bodyRates[1]*NavData.bodyRates[1] + NavData.bodyRates[2]*NavData.bodyRates[2]);
}

void performINS( double &navDt )
{    
    // Integrate Position (N, E, Alt)
    NavData.deltaPosNED[0] = NavData.velNED[0]*navDt + 0.5*NavData.deltaVelNED[0]*navDt;
    NavData.deltaPosNED[1] = NavData.velNED[1]*navDt + 0.5*NavData.deltaVelNED[1]*navDt;
    NavData.deltaPosNED[2] = -NavData.velNED[2]*navDt - 0.5*NavData.deltaVelNED[2]*navDt;
    
    NavData.position[0] = NavData.position[0] + NavData.velNED[0]*navDt + 0.5*NavData.deltaVelNED[0]*navDt;
    NavData.position[1] = NavData.position[1] + NavData.velNED[1]*navDt + 0.5*NavData.deltaVelNED[1]*navDt;
    NavData.position[2] = NavData.position[2] - NavData.velNED[2]*navDt - 0.5*NavData.deltaVelNED[2]*navDt;
    
    // height_above_ellipsoid = MSL + geoidHeight
    NavData.altitude_msl = NavData.position[2] - NavData.geoidCorrection;
    
    // Integrate Velocity
    for (int i=0; i<3; i++)
    {
        NavData.velNED[i] += NavData.deltaVelNED[i];
    }
}

void propogateVariance( double &navDt )
{
    // Intermidiate variables
    double PHItrans[NSTATES][NSTATES] = {0.0};
    double P_PHItrans[NSTATES][NSTATES] = {0.0};
    double PHI_P_PHItrans[NSTATES][NSTATES] = {0.0};
    double dVelIMU[3];
    
    double sr;
    double cr;
    double sp;
    double cp;
    double sy;
    double cy;
    double tp;
    double secp;
    double sec2p;
    double RH_h3;
    double dg_dh;
    double navDt2 = navDt*navDt;
    double gvar;
    
    NavType* pNav = 0;
    if (useTruthStateTransitionMatrix)
    {
#ifdef SIMULATION
        pNav = &truthNavData;
#else
        pNav = &NavData;
#endif
    }
    else
    {
        pNav = &NavData;
    }
    
    sr    = sin(pNav->eulerAngles[0]);
    cr    = cos(pNav->eulerAngles[0]);
    sp    = sin(pNav->eulerAngles[1]);
    cp    = cos(pNav->eulerAngles[1]);
    sy    = sin(pNav->eulerAngles[2]);
    cy    = cos(pNav->eulerAngles[2]);
    tp    = tan(pNav->eulerAngles[1]);
    secp  = 1.0/cos(pNav->eulerAngles[1]);
    sec2p = secp*secp;
    RH_h3 = (RE + pNav->position[2])*(RE + pNav->position[2])*(RE + pNav->position[2]);
    dg_dh = -2.0*GMe/RH_h3;
    
    for (int i = 0; i < 3; i++)
    {
        dVelIMU[i] = pNav->stateInputs.dVelocity[i] - pNav->gravityBody[i]*navDt;
    }
    
    // Roll
    PHI[ATT_X][ATT_X]   = 1.0 + cr*tp*pNav->stateInputs.dTheta[Y] - sr*tp*pNav->stateInputs.dTheta[Z];
    PHI[ATT_X][ATT_Y]   = sr*sec2p*pNav->stateInputs.dTheta[Y] + cr*sec2p*pNav->stateInputs.dTheta[Z];
    PHI[ATT_X][GBIAS_X] = -1.0*navDt;
    PHI[ATT_X][GBIAS_Y] = (-sr*tp)*navDt;
    PHI[ATT_X][GBIAS_Z] = (-cr*tp)*navDt;
    //Pitch
    PHI[ATT_Y][ATT_X]   = -sr*pNav->stateInputs.dTheta[Y]- cr*pNav->stateInputs.dTheta[Z];
    PHI[ATT_Y][ATT_Y]   = 1.0;
    PHI[ATT_Y][GBIAS_Y] = -cr*navDt;
    PHI[ATT_Y][GBIAS_Z] = sr*navDt;
    // Yaw
    PHI[ATT_Z][ATT_X]   = cr*secp*pNav->stateInputs.dTheta[Y] - sr*secp*pNav->stateInputs.dTheta[Z];
    PHI[ATT_Z][ATT_Y]   = secp*tp*sr*pNav->stateInputs.dTheta[Y] + secp*tp*cr*pNav->stateInputs.dTheta[Z];
    PHI[ATT_Z][ATT_Z]   = 1.0;
    PHI[ATT_Z][GBIAS_Y] = -sr*secp*navDt;
    PHI[ATT_Z][GBIAS_Z] = -cr*secp*navDt;
    
    // VN
    PHI[VN][ATT_X]   = (cy*sp*cr+sr*sy)*dVelIMU[Y] + (-cy*sp*sr+cr*sy)*dVelIMU[Z];
    PHI[VN][ATT_Y]   = -sp*cy*dVelIMU[X] + cy*cp*sr*dVelIMU[Y] + cy*cp*cr*dVelIMU[Z];
    PHI[VN][ATT_Z]   = -cp*sy*dVelIMU[X] + (-sy*sp*sr-cr*cy)*dVelIMU[Y] + (-sy*sp*cr+sr*cy)*dVelIMU[Z];
    PHI[VN][VN]      = 1.0;
    PHI[VN][ABIAS_X] = -cp*cy*navDt;
    PHI[VN][ABIAS_Y] = -(cy*sp*sr-cr*sy)*navDt;
    PHI[VN][ABIAS_Z] = -(cy*sp*cr+sr*sy)*navDt;
    // VE
    PHI[VE][ATT_X]   = (-sr*cy+sp*cr*sy)*dVelIMU[Y] + (-cr*cy-sp*sr*sy)*dVelIMU[Z];
    PHI[VE][ATT_Y]   = -sp*sy*dVelIMU[X] + cp*sr*sy*dVelIMU[Y] + cp*cr*sy*dVelIMU[Z];
    PHI[VE][ATT_Z]   = cp*cy*dVelIMU[X]+ + (-cr*sy+sp*sr*cy)*dVelIMU[Y] + (sr*sy+sp*cr*sy)*dVelIMU[Z];
    PHI[VE][VE]      = 1.0;
    PHI[VE][ABIAS_X] = -cp*sy*navDt;
    PHI[VE][ABIAS_Y] = -(cr*cy+sp*sr*sy)*navDt;
    PHI[VE][ABIAS_Z] = -(sr*cy+sp*cr*sy)*navDt;
    // VD
    PHI[VD][ATT_X]   = cp*cr*dVelIMU[Y] - cp*sr*dVelIMU[Z];
    PHI[VD][ATT_Y]   = -cp*dVelIMU[X] - sp*sr*dVelIMU[Y] - sp*cr*dVelIMU[Z];
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
    
    // Gravity Bias
    PHI[GRAVITY][GRAVITY] = 1.0;
    gvar = dg_dh * P[ALT][ALT] * dg_dh;

    math_mtran(*PHItrans, *PHI, NSTATES, NSTATES);
    
    // Update Process Noise
    memset(Q, 0, NSTATES*NSTATES*sizeof(double));
    Q[ATT_X][ATT_X] = imuToNavRateRatio*(gyroCov[X][X] + gyroCov[Y][Y] + gyroCov[Z][Z])*navDt2;
    Q[ATT_Y][ATT_Y] = imuToNavRateRatio*(gyroCov[X][X] + gyroCov[Y][Y] + gyroCov[Z][Z])*navDt2;
    Q[ATT_Z][ATT_Z] = imuToNavRateRatio*(gyroCov[X][X] + gyroCov[Y][Y] + gyroCov[Z][Z])*navDt2;
    Q[VN][VN]       = imuToNavRateRatio*sqrt(accelCov[X][X]*accelCov[X][X] + accelCov[Y][Y]*accelCov[Y][Y])*navDt2;
    Q[VE][VE]       = imuToNavRateRatio*sqrt(accelCov[X][X]*accelCov[X][X] + accelCov[Y][Y]*accelCov[Y][Y])*navDt2;
    Q[VD][VD]       = imuToNavRateRatio*accelCov[Z][Z]*navDt2 + gvar*navDt2;
    
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
    {
        filterUpdate(groundResidual, *R_GROUND, *H_GROUND, *K_GROUND, NGROUNDSTATES);
    }

    NavData.velNED[0] += stateError[VN];
    NavData.velNED[1] += stateError[VE];
    NavData.velNED[2] += stateError[VD];
    
    NavData.position[0] += stateError[N];
    NavData.position[1] += stateError[E];
    NavData.position[2] += stateError[ALT];

    NavData.eulerAngles[0] += stateError[ATT_X];
    NavData.eulerAngles[1] += stateError[ATT_Y];
    NavData.eulerAngles[2] += stateError[ATT_Z];
    updateQuaternions();

    NavData.gyroBias[0] += stateError[GBIAS_X];
    NavData.gyroBias[1] += stateError[GBIAS_Y];
    NavData.gyroBias[2] += stateError[GBIAS_Z];
    
    NavData.accBias[0] += stateError[ABIAS_X];
    NavData.accBias[1] += stateError[ABIAS_Y];
    NavData.accBias[2] += stateError[ABIAS_Z];
    
    NavData.gravityBias += stateError[GRAVITY];
    
    for (int i = 0; i < NSTATES; i++)
    {
        stateErrorLatch[i] = stateError[i];
        stateErrorAccum[i] += stateError[i];
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
            Pcorrection[i][j] = -K_H[i][j];
        }
    }
}

void FsNavigation_performAccelerometerUpdate(bool performUpdate)
{
    if (!navSetup || !nav_pIMUdata || !nav_pIMUdata->IMUgood) { return; }
    
    if(NavData.state != Nav_Startup && NavData.state != Calibration && NavData.state != INS)
    {
        NavData.skippedUpdateCount[AccelUpdate]++;
        return;
    }
    
    static int stable_count = 0;
    static bool variance_set = false;
    double accel_var[3];
    double M, M2, Myz, Myz2, M2Myz;
    double accel[3];
    double droll_dbias[3];
    double dpitch_dbias[3];
    
    for (int i=0; i<3; i++)
    {
        if (NavData.allowLoadIMUCal)
        {
            accel[i] = nav_pIMUdata->accel[i] - NavData.accBias[i];
        }
        else
        {
            accel[i] = nav_pIMUdata->accel[i];
        }
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

        accel_var[0] = accelCov[0][0] + P[ABIAS_X][ABIAS_X];
        accel_var[1] = accelCov[1][1] + P[ABIAS_Y][ABIAS_Y];
        accel_var[2] = accelCov[2][2] + P[ABIAS_Z][ABIAS_Z];
        
        R_ACCEL[ACCEL_ROLL][ACCEL_ROLL] = droll_dbias[0]*accel_var[0]*droll_dbias[0] + droll_dbias[1]*accel_var[1]*droll_dbias[1] + droll_dbias[2]*accel_var[2]*droll_dbias[2];
        R_ACCEL[ACCEL_PITCH][ACCEL_PITCH] = dpitch_dbias[0]*accel_var[0]*dpitch_dbias[0] + dpitch_dbias[1]*accel_var[1]*dpitch_dbias[1] + dpitch_dbias[2]*accel_var[2]*dpitch_dbias[2];

        if (NavData.state > Calibration) { variance_set = true; }
    }
    
    if (NavData.state != Nav_Startup && performUpdate && NavData.rates_mag < highRate && (NavData.accel_mag < 0.2 || groundFlag))
        //&& !nav_pIMUdata->highDynamics )
    {
        stable_count++;
        if (stable_count > 10)
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
    
    // Set to calibration on first pass
    if (NavData.state == Nav_Startup)
    {
        NavData.state = Calibration;
        
        NavData.eulerAngles[0] = NavData.accel_roll;
        NavData.eulerAngles[1] = NavData.accel_pitch;
        
#ifdef SIMULATION
        if (use_truth_attitude && pDyn)
        {
            NavData.eulerAngles[0] = pDyn->getEulerAngles()[0];
            NavData.eulerAngles[1] = pDyn->getEulerAngles()[1];
            NavData.eulerAngles[2] = pDyn->getEulerAngles()[2];
        }
#endif
        updateQuaternions();
        P[ATT_X][ATT_X] = R_ACCEL[ACCEL_ROLL][ACCEL_ROLL];
        P[ATT_Y][ATT_Y] = R_ACCEL[ACCEL_PITCH][ACCEL_PITCH];
    }
}

void FsNavigation_performGPSUpdate(const GpsType* gpsData)
{
    if ( (NavData.state <= Calibration) || (!navSetup) ) { return; }

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

void FsNavigation_performBarometerUpdate(const BarometerType* baroData)
{
    if ( (NavData.state <= Calibration) || (!navSetup) ) { return; }
    
    if(NavData.state != INS)
    {
        NavData.skippedUpdateCount[BaroUpdate]++;
        return;
    }
    
    if (baroData->baroGood)
    {
        R_BARO[BARO_ALT][BARO_ALT] = FsBarometer_getAltitudeVariance();
        baroResidual[BARO_ALT]     = baroData->altitude + baroRefAltitude - NavData.position[2];

        NavData.state = BaroUpdate;
        NavData.sensorTimestamp[NavData.state] = baroData->timestamp;
    }
}

void FsNavigation_groundAlign()
{
    if ( (NavData.state <= Calibration) || !navSetup || !groundFlag || movementFlag ) { return; }
    if(NavData.state != INS)
    {
        NavData.skippedUpdateCount[GroundAlign]++;
        return;
    }

    R_GROUND[GROUND_N][GROUND_N]     = errorToVariance(1.0);
    R_GROUND[GROUND_E][GROUND_E]     = errorToVariance(1.0);
    R_GROUND[GROUND_ALT][GROUND_ALT] = errorToVariance(1.0);
    R_GROUND[GROUND_VN][GROUND_VN]   = errorToVariance(0.1);
    R_GROUND[GROUND_VE][GROUND_VE]   = errorToVariance(0.1);
    R_GROUND[GROUND_VD][GROUND_VD]   = errorToVariance(0.1);
    R_GROUND[GROUND_YAW][GROUND_YAW] = yaw_var_ground_latch;
    
    groundResidual[GROUND_N]   = position_ground_latch[0] - NavData.position[0];
    groundResidual[GROUND_E]   = position_ground_latch[1] - NavData.position[1];
    groundResidual[GROUND_ALT] = position_ground_latch[2] - NavData.position[2];
    groundResidual[GROUND_VN]  = 0.0 - NavData.velNED[0];
    groundResidual[GROUND_VE]  = 0.0 - NavData.velNED[1];
    groundResidual[GROUND_VD]  = 0.0 - NavData.velNED[2];
    groundResidual[GROUND_YAW] = yaw_ground_latch - NavData.eulerAngles[2];
    
    NavData.state = GroundAlign;
    NavData.sensorTimestamp[NavData.state] = getTime();
}

void FsNavigation_calibrateIMU()
{
    static bool firstTime = true;
    static double gravity_body[3];
    static double g_error_est[3];
    
    if (NavData.state != Calibration || !nav_pIMUdata || !nav_pIMUdata->IMUgood) { return; }
    
    if (firstTime)
    {
        double roll_error, pitch_error;
        double sr, cr, sp, cp;
        
        for (int i=0; i<3; i++)
        {
            calGyroError[i].reset();
            calAccelError[i].reset();
        }
        
        sr = sin(NavData.eulerAngles[0]);
        cr = cos(NavData.eulerAngles[0]);
        sp = sin(NavData.eulerAngles[1]);
        cp = cos(NavData.eulerAngles[1]);
        
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
        calGyroError[i].update(nav_pIMUdata->gyro[i]);
        calAccelError[i].update(nav_pIMUdata->accel[i] + gravity_body[i]);
    }
    
    NavData.updateCount[Calibration]++;
    NavData.sensorTimestamp[Calibration] = getTime();
    
    // Compute statistics
    if (NavData.updateCount[Calibration] >= maxCalCounter)
    {
        for (int i=0; i<3; i++)
        {
            calGyroError[i].compute();
            calAccelError[i].compute();

            NavData.gyroBias[i] = calGyroError[i].mean;
            NavData.accBias[i]  = calAccelError[i].mean;
        }
        
        double gyroQuatizationVariance =  errorToVariance(nav_pIMUdata->gyroQuantizationError_dps*degree2radian);
        double accelQuatizationVariance = errorToVariance(nav_pIMUdata->accelQuantizationError_g*Gravity);
        
        gyroCov[0][0]  = calGyroError[0].variance + gyroQuatizationVariance;
        gyroCov[1][1]  = calGyroError[1].variance + gyroQuatizationVariance;
        gyroCov[2][2]  = calGyroError[2].variance + gyroQuatizationVariance;
        accelCov[0][0] = calAccelError[0].variance + accelQuatizationVariance;
        accelCov[1][1] = calAccelError[1].variance + accelQuatizationVariance;
        accelCov[2][2] = calAccelError[2].variance + accelQuatizationVariance;

        P[ABIAS_X][ABIAS_X] = errorToVariance(g_error_est[0]);
        P[ABIAS_Y][ABIAS_Y] = errorToVariance(g_error_est[1]);
        P[ABIAS_Z][ABIAS_Z] = errorToVariance(g_error_est[2]);
        P[GRAVITY][GRAVITY] = errorToVariance(0.0*Gravity);
        
        P[GBIAS_X][GBIAS_X] = errorToVariance(0.03*degree2radian);
        P[GBIAS_Y][GBIAS_Y] = errorToVariance(0.03*degree2radian);
        P[GBIAS_Z][GBIAS_Z] = errorToVariance(0.03*degree2radian);
        
#ifdef SIMULATION
        if (use_truth_bias && pImu)
        {
            NavData.gyroBias[0] = IMUtoBody[0] * pImu->getGyroscopeBiasDps()[0] * degree2radian;
            NavData.gyroBias[1] = IMUtoBody[1] * pImu->getGyroscopeBiasDps()[1] * degree2radian;
            NavData.gyroBias[2] = IMUtoBody[2] * pImu->getGyroscopeBiasDps()[2] * degree2radian;
            
            NavData.accBias[0]  = IMUtoBody[0] * pImu->getAccelerometerBiasGs()[0] * Gravity;
            NavData.accBias[1]  = IMUtoBody[1] * pImu->getAccelerometerBiasGs()[1] * Gravity;
            NavData.accBias[2]  = IMUtoBody[2] * pImu->getAccelerometerBiasGs()[2] * Gravity;
            
            P[ABIAS_X][ABIAS_X] = 0.0;
            P[ABIAS_Y][ABIAS_Y] = 0.0;
            P[ABIAS_Z][ABIAS_Z] = 0.0;
            
            P[GBIAS_X][GBIAS_X] = 0.0;
            P[GBIAS_Y][GBIAS_Y] = 0.0;
            P[GBIAS_Z][GBIAS_Z] = 0.0;
        }
#endif
        if (NavData.allowLoadIMUCal)
        {
            NavData.gyroBias[0] = 0.0;
            NavData.gyroBias[1] = 0.0;
            NavData.gyroBias[2] = 0.0;
            
            NavData.accBias[0]  = 0.0;
            NavData.accBias[1]  = 0.0;
            NavData.accBias[2]  = 0.0;
            
            P[ABIAS_X][ABIAS_X] = 0.0;
            P[ABIAS_Y][ABIAS_Y] = 0.0;
            P[ABIAS_Z][ABIAS_Z] = 0.0;
        }
        
        NavData.state = INS;
    }
}

const NavType* FsNavigation_getNavData(bool useTruth)
{
#ifdef SIMULATION
    if (useTruth) { return &truthNavData; }
    else { return &NavData; }
#else
    return &NavData;
#endif
}

#ifdef SIMULATION
const NavType* FsNavigation_getNavError()
{
    return &NavError;
}
#endif

#ifdef SIMULATION
const NavType* FsNavigation_getTruthNavData()
{
    return &truthNavData;
}
#endif

NavState FsNavigation_getNavState()
{
    return NavData.state;
}

double FsNavigation_getNavAlt()
{
    return NavData.position[2];
}

const double* FsNavigation_getCovariance()
{
    return *P;
}

const double* FsNavigation_getProcessNoise()
{
    return *Q;
}

const double* FsNavigation_getStateTransition()
{
    return *PHI;
}

const double* FsNavigation_getStateError()
{
    return stateErrorLatch;
}
const double* FsNavigation_getAccumStateError()
{
    return stateErrorAccum;
}

const double* FsNavigation_getCovarianceCorrection()
{
    return *Pcorrection;
}

const double* FsNavigation_getBaroKalmanGain()
{
    return *K_BARO;
}

const double* FsNavigation_getBaroResidual()
{
    return baroResidual;
}

const double* FsNavigation_getGPSKalmanGain()
{
    return *K_GPS;
}

const double* FsNavigation_getGPSMeasVariance()
{
    return *R_GPS;
}

const double* FsNavigation_getGPSResidual()
{
    return gpsResidual;
}

const double* FsNavigation_getAccelMeasVariance()
{
    return *R_ACCEL;
}

const double* FsNavigation_getGroundResidual()
{
    return groundResidual;
}
const SensorErrorType* FsNavigation_getGyroStatistics()
{
    return &calGyroError[0];
}

const SensorErrorType* FsNavigation_getAccelStatistics()
{
    return &calAccelError[0];
}

#ifdef SIMULATION
void FsNavigation_setSimulationModels(ModelMap* pMap)
{
    pDyn  = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pImu  = (IMUModelBase*)    pMap->getModel("IMUModel");
    pAtmo = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
}

void FsNavigation_setTruthTransitionMatrix()
{
    useTruthStateTransitionMatrix = true;
}

#endif

void FsNavigation_setIMUToNavRateRatio(double rateRatio)
{
    imuToNavRateRatio = rateRatio;
}

void FsNavigation_setIMUdata(const IMUtype* pIMUdataIn)
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

void FsNavigation_setGroundFlags(bool onGround, bool movingDetection)
{
    if ( (!groundFlag && onGround) || (groundFlag && movementFlag && !movingDetection) )
    {
        // Reset position and yaw latch if
        //   1) landing back on ground
        //   2) stop after moving on ground
        position_ground_latch[0] = NavData.position[0];
        position_ground_latch[1] = NavData.position[1];
        position_ground_latch[2] = NavData.position[2];
        
        yaw_var_ground_latch = P[ATT_Z][ATT_Z];
        yaw_ground_latch = NavData.eulerAngles[2];
        if (yaw_var_ground_latch*yaw_var_ground_latch < YAWSTDLIM)
        {
            H_GROUND[GROUND_YAW][ATT_Z] = 1.0;
        }
    }
    groundFlag = onGround;
    movementFlag = movingDetection;
}

void FsNavigation_setVelocityCorrectionsFlag(bool flag)
{
    nav_apply_velocity_corrections = flag;
}

#ifdef SIMULATION
void updateTruth()
{
    if (pDyn)
    {
        truthNavData.position[0] = pDyn->getPosNED()[0];
        truthNavData.position[1] = pDyn->getPosNED()[1];
        truthNavData.position[2] = -pDyn->getPosNED()[2];
        
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
        
        truthNavData.accel_mag = sqrt(truthNavData.accelBody[0]*truthNavData.accelBody[0] +
                                      truthNavData.accelBody[1]*truthNavData.accelBody[1] +
                                      truthNavData.accelBody[2]*truthNavData.accelBody[2]);
        
        truthNavData.stateInputs.dVelocity[0] = pDyn->getDeltaVelocity()[0];
        truthNavData.stateInputs.dVelocity[1] = pDyn->getDeltaVelocity()[1];
        truthNavData.stateInputs.dVelocity[2] = pDyn->getDeltaVelocity()[2];
   
        truthNavData.deltaVelNED[0] = pDyn->getDeltaVelocityNED()[0];
        truthNavData.deltaVelNED[1] = pDyn->getDeltaVelocityNED()[1];
        truthNavData.deltaVelNED[2] = pDyn->getDeltaVelocityNED()[2];
        
        truthNavData.deltaPosNED[0] = pDyn->getDeltaPositionNED()[0];
        truthNavData.deltaPosNED[1] = pDyn->getDeltaPositionNED()[1];
        truthNavData.deltaPosNED[2] = -pDyn->getDeltaPositionNED()[2];
        
        truthNavData.bodyRates[0] = pDyn->getBodyRates()[0];
        truthNavData.bodyRates[1] = pDyn->getBodyRates()[1];
        truthNavData.bodyRates[2] = pDyn->getBodyRates()[2];
        
        truthNavData.stateInputs.dTheta[0] = pDyn->getDeltaTheta()[0];
        truthNavData.stateInputs.dTheta[1] = pDyn->getDeltaTheta()[1];
        truthNavData.stateInputs.dTheta[2] = pDyn->getDeltaTheta()[2];
        
        truthNavData.imuTimestamp = pDyn->getTimestamp();
    }
    
    if (pImu)
    {
        truthNavData.accBias[0] = pImu->getAccelerometerBiasGs()[0] * Gravity;
        truthNavData.accBias[1] = pImu->getAccelerometerBiasGs()[1] * Gravity;
        truthNavData.accBias[2] = pImu->getAccelerometerBiasGs()[2] * Gravity;
        
        truthNavData.gyroBias[0] = pImu->getGyroscopeBiasDps()[0];
        truthNavData.gyroBias[1] = pImu->getGyroscopeBiasDps()[1];
        truthNavData.gyroBias[2] = pImu->getGyroscopeBiasDps()[2];
    }
        
    if (pAtmo)
    {
        truthNavData.gravity = pAtmo->getGravity();
        truthNavData.gravityBody[0] = pAtmo->getGravityBody()[0];
        truthNavData.gravityBody[1] = pAtmo->getGravityBody()[1];
        truthNavData.gravityBody[2] = pAtmo->getGravityBody()[2];
        
        truthNavData.accel_roll  = pAtmo->getGravityRoll();
        truthNavData.accel_pitch = pAtmo->getGravityPitch();
    }
}

void computeTruthErrors()
{
    static double sum_deltaVelNED[3];
    static int count_deltaVelNED[3];
    for (int i=0; i<3; i++)
    {
        NavError.position[i]    = NavData.position[i]     - truthNavData.position[i];
        NavError.velNED[i]      = NavData.velNED[i]       - truthNavData.velNED[i];
        NavError.eulerAngles[i] = (NavData.eulerAngles[i] - truthNavData.eulerAngles[i]) * radian2degree;
        NavError.q_B_NED[i]     = NavData.q_B_NED[i]      - truthNavData.q_B_NED[i];
        NavError.accelBody[i]   = NavData.accelBody[i]    - truthNavData.accelBody[i];
        NavError.bodyRates[i]   = (NavData.bodyRates[i]   - truthNavData.bodyRates[i]) * radian2degree;
        NavError.gravityBody[i] = NavData.gravityBody[i]  - truthNavData.gravityBody[i];
        
        NavError.stateInputs.dTheta[i]    = NavData.stateInputs.dTheta[i] - truthNavData.stateInputs.dTheta[i];
        NavError.stateInputs.dVelocity[i] = NavData.stateInputs.dVelocity[i] - truthNavData.stateInputs.dVelocity[i];
        NavError.deltaVelNED[i]           = NavData.deltaVelNED[i] - truthNavData.deltaVelNED[i];
        NavError.deltaPosNED[i]           = NavData.deltaPosNED[i] - truthNavData.deltaPosNED[i];
        
        NavError.gyroBias[i] = (IMUtoBody[i]*NavData.gyroBias[i]*radian2degree - truthNavData.gyroBias[i]);
        NavError.accBias[i]  = (IMUtoBody[i]*NavData.accBias[i]- truthNavData.accBias[i])/(Gravity);
        
        if (NavError.eulerAngles[i] > 180.0) { NavError.eulerAngles[i] -= 360.0; }
        else if (NavError.eulerAngles[i] < -180.0) { NavError.eulerAngles[i] += 360.0; }
        
        if (fabs(NavError.deltaVelNED[i]) > 0.000001)
        {
            sum_deltaVelNED[i] += fabs(NavError.deltaVelNED[i]);
            count_deltaVelNED[i]++;
        }
    }
    /*
    std::cout << "mean_deltaVelNED_Error = [";
    std::cout << sum_deltaVelNED[0]/count_deltaVelNED[0] << ", ";
    std::cout << sum_deltaVelNED[1]/count_deltaVelNED[1] << ", ";
    std::cout << sum_deltaVelNED[2]/count_deltaVelNED[2] << "]" << std::endl;
    */
    NavError.q_B_NED[3] = NavData.q_B_NED[3] - truthNavData.q_B_NED[3];
    
    NavError.gravity   = NavData.gravity - truthNavData.gravity;
    NavError.accel_mag = NavData.accel_mag - truthNavData.accel_mag;
    
    NavError.accel_roll  = (NavData.accel_roll  - truthNavData.accel_roll ) * radian2degree;
    NavError.accel_pitch = (NavData.accel_pitch - truthNavData.accel_pitch) * radian2degree;
    
    NavError.imuTimestamp = (NavData.imuTimestamp - truthNavData.imuTimestamp)*1000.0;
}

void copmuteObservability(ObservabilityTestType oInfo)
{
    // Get subset of states to be tested
    unsigned int nstates = 0;
    for (int i = 0; i < NSTATES; i++)
    {
        if (oInfo.tested[i]) { nstates++; }
    }
    
    if (nstates == 0) { return; }
    
    // Get A and C from subset of states
    double A[nstates][nstates];
    double C[oInfo.nMeas][nstates];
    
    double Atrans[nstates][nstates];
    double Ctrans[nstates][oInfo.nMeas];
    
    unsigned int xstate = 0;
    unsigned int ystate = 0;
    for (int i = 0; i < NSTATES; i++)
    {
        if (oInfo.tested[i])
        {
            for (int j = 0; j < oInfo.nMeas; j++)
            {
                C[j][xstate] = *(oInfo.H+j*oInfo.nMeas+i);
            }
            
            ystate = 0;
            for (int j = 0; j < NSTATES; j++)
            {
                if (oInfo.tested[j])
                {
                    A[xstate][ystate++] = PHI[i][j];
                }
            }
            xstate++;
        }
    }
    
    // O = [C' A'C' (A')^2 C' ... (A')^(n-1) C'] = [nstates, nMeas*(nstates-1)]
    math_mtran(*Atrans, *A, nstates, nstates);
    math_mtran(*Ctrans, *C, nstates, oInfo.nMeas);
    double O[nstates][oInfo.nMeas*(nstates-1)];
    unsigned int nAC = oInfo.nMeas * nstates;
    
    //Aij = *(matrix+i*ncol+j);
    for (int i = 0; i < nstates; i++)
    {
        double AnC[nstates][oInfo.nMeas];
        double An[nstates][nstates];
        for (int ii=0;ii<nstates;ii++)
        {
            for (int jj=0; jj<nstates;jj++)
            {
                if (ii==jj) { An[ii][jj] = 1.0; }
                else { An[ii][jj] = 0.0; }
            }
        }
        
        for (int j = 0; j < i; j++)
        {
            double Aresults[nstates][nstates];
            math_mmult(*Aresults, *An, nstates, nstates, *Atrans, nstates, nstates);
            memcpy(*An, *Aresults, nstates*nstates*sizeof(double));
        }
        math_mmult(*AnC, *An, nstates, nstates, *Ctrans, nstates, oInfo.nMeas);
        
        memcpy(*(O+i*oInfo.nMeas), *AnC, nAC*sizeof(double));
    }
}
#endif

inline void FsNavigation_bodyToNED(double* vNED, const double* vB)
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

void FsNavigation_NEDToBody(double* vB, const double* vNED)
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

void FsNavigation_NEDToLL(double* vLL, const double* vNED)
{
    double sy = sin(NavData.eulerAngles[2]);
    double cy = cos(NavData.eulerAngles[2]);
    
    vLL[0] =   cy*vNED[0] + sy*vNED[1];
    vLL[1] =  -sy*vNED[0] + cy*vNED[1];
    vLL[2] = vNED[2];
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
    const double* q = NavData.q_B_NED;
    NavData.eulerAngles[0] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1.0);
    NavData.eulerAngles[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    NavData.eulerAngles[2] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1.0);
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
                *(result+i*ncols2+j) += Aik*Bkj; // C[i][j] = A[i][k]*B[k][j]
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
