//
//  flight_software.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "flight_software.hpp"
#include "fs_telemetry.hpp"
#include "fs_imu.hpp"
#include "fs_pwmin.hpp"
#include "fs_controls.hpp"
#include "fs_thrust_estimator.hpp"
#include "fs_navigation.hpp"
#include "fs_gps.hpp"

#ifdef SIMULATION
    #include "dynamics_model.hpp"
#endif

// General Settings
int baudRate;
bool intiailized = false;

// Print
#ifdef PRINT
    bool printTiming;
    bool printIMU;
    bool printCalibration;
    bool printCalibratedIMU;
    bool printBarometer;
    bool printGPS;
    bool printGPS_DOP;
    bool printNavInputs;
    bool printAngles;
    bool printVelocity;
    bool printPosition;
    bool printNavVariance;
    bool printAccelFilter;
    bool printNavFilterCount;
    bool printPWMIn;
    bool printCommands;
    bool printMotorPWM;
    bool printTakeOff;
    bool printTelemetry;
#endif

// Simulation Classes
#ifdef SIMULATION
    class ModelMap* pMap = 0;
    class DynamicsModel* pDynModel = 0;
#endif

// Time keeping (s)
double prevTime[nRoutines];
double routineDelays[nRoutines];
bool   performRoutine[nRoutines];
double actualDelays[nRoutines];

// Event keeping (s)
enum {imuWarmup, nEvents};
double eventStartTimes[nEvents];
bool eventStarted[nEvents];

// IMU
const IMUtype* pIMUdata = 0;
accSensitivityType accSensitivity;
gyroSensitivityType gyroSensitivity;

// Barometer
const BarometerType* pBaroData = 0;
byte bmp180Resolution;
bool useBarometer;

// GPS
const GpsType* pGPSdata = 0;
bool fs_allow2DFix;
int gpsBaudRate;
bool useGPS;

// Navigation
bool useTruthNav;
bool useTruthStateTran;
double initialPosition[3];
double initialHeading;
bool useAccelPitchRoll;
bool groundAlign;
bool loadIMUCalibration;
const NavType* pNavData = 0;
#ifdef SIMULATION
    const NavType* pNavError = 0;
#endif

// Controls
const ControlType* pControlData = 0;
ControlMode controlMode;

// Thrust Estimator
const ThrustEstimatorType* pThrustEstData = 0;
#ifdef SIMULATION
    const ThrustEstimatorType* pThrustError = 0;
#endif

// Telemetry
const TMType* pTMData = 0;
int TMBaudRate;
bool configTM;
float TM_MSG_Rates[N_TM_MSGS];
unsigned int TM_MSG_Counts[N_TM_MSGS];

// Print
#ifdef SIMULATION
    double highDynamics = 0.0;
    double gyro_dps[3];
    double dTheta_deg[3];
    double eulerAnglesDeg[3];
    double gyroBias_dps[3];
    double bodyRates_dps[3];
    double nav_dTheta_deg[3];
    double accBias_g[3];
    double nav_accel_pitch;
    double nav_accel_roll;
    double nav_velLL[3];
    double nav_velLL_error[3];
    double nav_accelLL[3];
    double nav_accelLL_error[3];
    double fsTime;
    double countDelta50hz;
    double navState = 0.0;
    double NavUpdateCounts[NNAVSTATES];
    double NavSkippedUpdates[NNAVSTATES];
    double gpsGood = 0.0;
    double gpsFix = 0.0;
    double fixOk  = 0.0;
    double numSV  = 0.0;
    double gps_rcvdByteCount = 0.0;
    double gps_rcvdMsgCount = 0.0;
    double gps_fifoWritedByteCount = 0.0;
    double altitudeError = 0.0;
    double cpwmCmd;
    double rollCmd;
    double pitchCmd;
    double yawRateCmd;
    double controlAltitude = 0.0;
    double ctrlMode = 0.0;
    double takeOff   = 0.0;
    double crashLand = 0.0;
    double onGround  = 1.0;
    double movingDetection = 0.0;
    double daMomentCmd = 0.0;
    double deMomentCmd = 0.0;
    double drMomentCmd = 0.0;
    double baroState = 0.0;
    double motor_dTheta_deg[3];

    double Cov[NSTATES][NSTATES];
    double PN[NSTATES][NSTATES];
    double KB[NSTATES][NBAROSTATES];
    double filterStateError[NSTATES];

    double RGPS[NGPSSTATES][NGPSSTATES];
    double stdGPS[NGPSSTATES][NGPSSTATES];

    double covarianceCorrection[NSTATES][NSTATES];
    double barometerResidual[NBAROSTATES];
    double navStd[NSTATES][NSTATES];
    double nav3Std[NSTATES][NSTATES];

    double GPSResidual[NGPSSTATES];
    double GPSFilterError[NGPSSTATES];

    double RAccel[NACCELSTATES][NACCELSTATES];
    double stdAccel[NACCELSTATES][NACCELSTATES];
#endif

// **********************************************************************
// Initialize
// **********************************************************************
void initialize(void)
{
    initializeRoutines();

    // Print Settings
#ifdef PRINT
    printTiming        = true;
    printIMU           = false;
    printCalibration   = false;
    printCalibratedIMU = false;
    printBarometer     = false;
    printGPS           = true;
    printGPS_DOP       = false;
    printNavInputs     = false;
    printAngles        = false;
    printVelocity      = false;
    printPosition      = false;
    printNavVariance   = false;
    printAccelFilter    = false;
    printNavFilterCount = false;
    printPWMIn         = false;
    printCommands      = false;
    printMotorPWM      = false;
    printTakeOff       = false;
    printTelemetry     = true;
#endif
    
    // General Settings
    baudRate = 9600; // 9600, 38400, 115200
    
    // IMU Settings
    accSensitivity  = accSensitivity_16g;
    gyroSensitivity = gyroSensitivity_250dps;
    
    // Barometer Settings
    bmp180Resolution = BMP180_COMMAND_PRESSURE3;
    useBarometer     = true;
    
    // Navigation Settings
    initialPosition[0] = 0.0; // North    (m)
    initialPosition[1] = 0.0; // East     (m)
    initialPosition[2] = 0.0; // Altitude (m)
    initialHeading     = 0.0;
    useAccelPitchRoll  = false;
    groundAlign        = true;
    loadIMUCalibration = false;
    useTruthNav        = false;
    useTruthStateTran  = false;
    
    // GPS
    fs_allow2DFix = false;
    gpsBaudRate   = 9600;
    useGPS        = true;
    
    // Control Settings
    controlMode = AttitudeControl; //NoControl, ThrottleControl, AttitudeControl, VelocityControl
    
    // Telemetry
    // Note: Rates must be divisible by 600
    TMBaudRate = 9600;
    configTM   = true;
    
    TM_MSG_Rates[FS_TM_IMU]          = 0.0;
    TM_MSG_Rates[FS_TM_BARO]         = 0.0;
    TM_MSG_Rates[FS_TM_GPS]          = 1.0;
    TM_MSG_Rates[FS_TM_NAV_HIGHRATE] = 0.0;
    TM_MSG_Rates[FS_TM_NAV_LOWRATE]  = 0.0;
    TM_MSG_Rates[FS_TM_CONTROLS]     = 0.0;
    TM_MSG_Rates[FS_TM_STATUS]       = 0.5;
    TM_MSG_Rates[FS_PRINT]           = 0.0;
    for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++ )
    { TM_MSG_Counts[tm_id] = 0;}
    
    // Timing Settings
    routineDelays[hz1]   = 1.0;
    routineDelays[hz50]  = 1.0/50.0;
    routineDelays[hz100] = 1.0/100.0;
    routineDelays[hz200] = 1.0/200.0;
    routineDelays[hz600] = 1.0/600.0;
    routineDelays[printRoutine] = 1.0;
    
    // Event Settings
    eventStartTimes[imuWarmup] = 1.0;
    
    // Finish Setup
    getModels();
    setupIO();
    initializeTime();
    
    LEDon();
}

// **********************************************************************
// Main Flight Software
// **********************************************************************
bool mainFlightSoftware(void)
{
    if (!intiailized)
    {
        initialize();
        intiailized = true;
    }

    // Update Event Booleans
    for (int i=0; i<nEvents; i++)
    {
        eventStarted[i] = getTime() >= eventStartTimes[i];
    }
    
    // Update Routine Booleans
    for (int i=0; i<nRoutines; i++)
    {
        performRoutine[i] = getTime() + dtPad - prevTime[i] >= routineDelays[i];
    }

    // Serial Routine
    performSerialIO();
    
    // 600 routine
    if (performRoutine[hz600])
    {
        // Timing info
        actualDelays[hz600] = getTime() - prevTime[hz600];
        prevTime[hz600] = getTime();
        
        // Update IMU
        FsImu_performIMU( actualDelays[hz600] );
        FsThrustEstimator_setGravity(pNavData->gravityBody);
        FsThrustEstimator_setBodyVelocity(pNavData->velBody);
        FsThrustEstimator_setMotorPWMCmd(pControlData->TPWM);
        FsThrustEstimator_rpmUpdate( actualDelays[hz600] );
        FsThrustEstimator_perform( actualDelays[hz600] );
        
        // Update Telemetry
        if (eventStarted[imuWarmup])
        {
            for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++ )
            {
                TM_MSG_Counts[tm_id]++;
                if (TM_MSG_Rates[tm_id] != 0 && TM_MSG_Counts[tm_id] >= 600/TM_MSG_Rates[tm_id])
                {
                    FsTelemetry_sendTelemetry((TM_MSG_TYPE) tm_id);
                    TM_MSG_Counts[tm_id] = 0;
                }
            }
        
            FsTelemetry_performTelemetry( actualDelays[hz600] );
        }
    }
    
    // Wait for IMU warmup
    if (!eventStarted[imuWarmup]) { return false; }
    
    // 200 hz routine
    if (performRoutine[hz200])
    {
        actualDelays[hz200] = getTime() - prevTime[hz200];
        prevTime[hz200] = getTime();
        
        // Calibrate Navigation
        FsNavigation_calibrateIMU();
        
        // Ground Detection
        if (pNavData->state > Calibration)
        {
            FsControls_setIMUStatistics(FsNavigation_getGyroStatistics(), FsNavigation_getAccelStatistics());
        }
        FsControls_setControlsData(pIMUdata, pNavData);
        FsControls_groundDetection();
        
        FsNavigation_setGroundFlags(pControlData->onGround, pControlData->movingDetection);
    }
    
    if (performRoutine[hz100])
    {
        actualDelays[hz100] = getTime() - prevTime[hz100];
        prevTime[hz100] = getTime();
        
        // Update Inertial Navigation
        FsNavigation_performNavigation( actualDelays[hz100] );
        
        // Perform Velocity and Attitude Control
        FsControls_performControls( actualDelays[hz100] );
        
        // Update thrust estimator kalman filter
        FsThrustEstimator_performIMUupdate(pNavData->stateInputs.dTheta, pNavData->stateInputs.dVelocity);
        
        FsImu_zeroDelta();
    }
    
    // 50 hz routine
    static unsigned int count5hz = 0;
    if (performRoutine[hz50])
    {
        actualDelays[hz50] = getTime() - prevTime[hz50];
        prevTime[hz50] = getTime();
        
        // Read Controller PWM
        FsPwmIn_performPwmIn();
        
        // Perform Guidance
  
        // Set commands
        if (FsPwmIn_valid())
        {
            FsControls_setPWMCommands( FsPwmIn_getPWM() );
        }
        
        // Update Barometer
        FsBarometer_performBarometer();
        if (FsBarometer_getBaroState() == baroReady)
        {
            if (useBarometer) { FsNavigation_performBarometerUpdate(pBaroData); }
            FsBarometer_setStandby();
        }
        
        // Update GPS
        FsGPS_performGPS();
        if (useGPS) { FsNavigation_performGPSUpdate(pGPSdata); }
        
        // 5Hz
        count5hz++;
        if (count5hz == 10)
        {
            // Acceleromter update
            FsNavigation_performAccelerometerUpdate(useAccelPitchRoll);
            count5hz = 0;
        }
    }
    
    static int count30sec = 0;
    if (performRoutine[hz1])
    {
        //Timing Info
        actualDelays[hz1] = getTime() - prevTime[hz1];
        prevTime[hz1] = getTime();
    
        controlsModing();
        
        // Start barometer measurements
        FsBarometer_startBarometerMeasurement();
        
        // Ground align Navigation
        if (groundAlign) { FsNavigation_groundAlign(); }
        
        // Reset moving detection
        FsControls_resetMovingDetection();
        
        count30sec++;
        if (count30sec == 30)
        {
            FsGPS_requestAidningInfo();
            count30sec = 0;
        }
    }
    
    if (performRoutine[printRoutine])
    {
        //Timing Info
        actualDelays[printRoutine] = getTime() - prevTime[printRoutine];
        prevTime[printRoutine] = getTime();
        
        printData();
    }
    
#ifdef SIMULATION
    // Print Variables
    fsTime = getTime();
    
    highDynamics      = static_cast<double> (pIMUdata->highDynamics);
    navState          = static_cast<double> (pNavData->state);
    
    for (int i = 0; i < NNAVSTATES; i++)
    {
        NavUpdateCounts[i] = pNavData->updateCount[i];
        NavSkippedUpdates[i] = pNavData->skippedUpdateCount[i];
    }
    
    baroState         = static_cast<double> (pBaroData->state);
    gpsGood           = static_cast<double> (pGPSdata->gpsGood);
    gpsFix            = static_cast<double> (pGPSdata->gpsFix);
    fixOk             = static_cast<double> (pGPSdata->fixOk);
    numSV             = static_cast<double> (pGPSdata->numSV);
    gps_rcvdByteCount = static_cast<double> (pGPSdata->rcvdByteCount);
    gps_rcvdMsgCount  = static_cast<double> (pGPSdata->rcvdMsgCount);
    gps_fifoWritedByteCount  = static_cast<double> (pGPSdata->fifoWriteCount);
    
    nav_accel_pitch = pNavData->accel_pitch * radian2degree;
    nav_accel_roll  = pNavData->accel_roll  * radian2degree;
    
    for (int i=0; i<3; i++)
    {
        dTheta_deg[i]     = pIMUdata->dTheta[i] * radian2degree;
        gyro_dps[i]       = pIMUdata->gyro[i] * radian2degree;
        eulerAnglesDeg[i] = pNavData->eulerAngles[i]*radian2degree;
        gyroBias_dps[i]   = pNavData->gyroBias[i]*radian2degree;
        bodyRates_dps[i]  = pNavData->bodyRates[i]*radian2degree;
        accBias_g[i]      = pNavData->accBias[i] / (Gravity);
        nav_dTheta_deg[i] = pNavData->stateInputs.dTheta[i]*radian2degree;
        motor_dTheta_deg[i] = pThrustEstData->deltaAngle[i]*radian2degree;
    }

    pNavError = FsNavigation_getNavError();
    altitudeError = -pNavError->position[2];
    FsNavigation_bodyToLL(nav_velLL, pNavData->velBody);
    FsNavigation_bodyToLL(nav_accelLL, pNavData->accelBody);
    
    if (pDynModel)
    {
        for (int i=0; i<3; i++)
        {
            nav_velLL_error[i] = nav_velLL[i] - pDynModel->getVelLL()[i];
            nav_accelLL_error[i] = nav_accelLL[i] - pDynModel->getAccLL()[i];
        }
    }
    
    cpwmCmd = (double) pControlData->pwmCmd[THROTTLE_CHANNEL];
    rollCmd    = pControlData->rollCmd * radian2degree;
    pitchCmd   = pControlData->pitchCmd * radian2degree;
    yawRateCmd = pControlData->yawRateCmd * radian2degree;
    controlAltitude = (double) pControlData->controlAltitude;
    ctrlMode  = (double) pControlData->mode;
    takeOff   = (double) pControlData->takeOff;
    crashLand = (double) pControlData->crashLand;
    onGround  = (double) pControlData->onGround;
    movingDetection = (double) pControlData->movingDetection;
    daMomentCmd = (KT)*PropDistance*pControlData->da;
    deMomentCmd = (KT)*PropDistance*pControlData->de;
    drMomentCmd = (KH)*pControlData->dr;
    
    const double* covCorTemp = FsNavigation_getCovarianceCorrection();
    for (int i = 0; i < NSTATES; i++)
    {
        for (int j = 0; j < NSTATES; j++)
        {
            covarianceCorrection[i][j] = *(covCorTemp + i*NSTATES + j);
        }
    }
    
    const double* Ptemp = FsNavigation_getCovariance();
    const double* Qtemp = FsNavigation_getProcessNoise();
    const double* Ktemp = FsNavigation_getBaroKalmanGain();
    const double* stemp = FsNavigation_getStateError();
    const double* RGPStemp = FsNavigation_getGPSMeasVariance();
    const double* RAcceltemp = FsNavigation_getAccelMeasVariance();
    
    double sgn = 1.0;
    for (int i=0; i<NSTATES; i++)
    {
        filterStateError[i] = *(stemp+i);
        for (int j=0; j<NSTATES; j++)
        {
            if (j < NBAROSTATES){ KB[i][j] = *(Ktemp+i*NBAROSTATES+j); }
            PN[i][j]      = *(Qtemp+i*NSTATES+j);
            Cov[i][j]     = *(Ptemp+i*NSTATES+j);
            
            if (i<NGPSSTATES && j<NGPSSTATES)
            {
                RGPS[i][j] = *(RGPStemp+i*NGPSSTATES+j);
                if (RGPS[i][j] < 0.0) { sgn = -1.0; }
                else { sgn = 1.0; }
                stdGPS[i][j] = sgn*sqrt(sgn*RGPS[i][j]);
            }
            
            if (i<NACCELSTATES && j<NACCELSTATES)
            {
                RAccel[i][j] = *(RAcceltemp+i*NACCELSTATES+j);
                if (RAccel[i][j] < 0.0) { sgn = -1.0; }
                else { sgn = 1.0; }
                stdAccel[i][j] = sgn*sqrt(sgn*RAccel[i][j]) / (degree2radian);
            }
            
            if (Cov[i][j] < 0) { sgn = -1.0; }
            else { sgn = 1.0; }
            navStd[i][j]  = sgn*sqrt(sgn*Cov[i][j]);
            
            if ((i <= YAW) || (j <= YAW))
            {
                navStd[i][j] = navStd[i][j] * radian2degree;
            }
            if ((i >= ABIAS_X && i <= ABIAS_Z) || (j >= ABIAS_X && j <= ABIAS_Z))
            {
                navStd[i][j] = navStd[i][j] / (Gravity);
            }
            nav3Std[i][j] = 3.0*navStd[i][j];
        }
    }
    
    const double* baroResidualTemp = FsNavigation_getBaroResidual();
    for (int i = 0; i < NBAROSTATES; i++)
    {
        barometerResidual[i] = baroResidualTemp[i];
    }

    for (int i = 0; i < NGPSSTATES; i++)
    {
        GPSResidual[i] = FsNavigation_getGPSResidual()[i];
        GPSFilterError[i] = (double) FsNavigation_getGPSFilterError()[i];
    }
    
#endif
    
    return true;
}

// **********************************************************************
// Initialization Functions
// **********************************************************************
void getModels()
{
    // Get Pointers
    pIMUdata  = FsImu_getIMUdata();
    pBaroData = FsBarometer_getBaroData();
    pGPSdata  = FsGPS_getGPSdata();
    pNavData  = FsNavigation_getNavData(useTruthNav);
    pControlData = FsControls_getControlData();
    pThrustEstData = FsThrustEstimator_getThrustEstimatorData();
    pTMData = FsTelemetry_getTMdata();
    
    FsNavigation_setIMUdata(pIMUdata);
    FsTelemetry_setTelemetryData(pIMUdata, pBaroData, pGPSdata, pNavData, pControlData);
    
# ifdef SIMULATION
    pNavError = FsNavigation_getNavError();
    pThrustError = FsThrustEstimator_getThrustEstimatorError();
    
    if(pMap)
    {
        // Set Simulation Pointers
        FsCommon_setSimulationModels(pMap);
        FsNavigation_setSimulationModels(pMap);
        FsControls_setSimulationModels(pMap);
        FsThrustEstimator_setSimulationModels(pMap);
        
        FsImu_setSimulationModels(pMap);
        FsGPS_setSimulationModels(pMap); // Send SoftwareSerial pointer to gps_model
        FsTelemetry_setSimulationModels(pMap);
        
        pDynModel = (DynamicsModel*) pMap->getModel("DynamicsModel");
        
        setPrintVariables();
    }
    else
    {
        display("Warning: Could not find map models.");
        display("\n");
    }
#endif
}

#ifdef SIMULATION
void flightSoftware_setMapPointer(ModelMap* pMapInit)
{
    pMap = pMapInit;
}
#endif

#ifdef SIMULATION
void setPrintVariables()
{
    // Timing
    //pMap->addLogVar("fsTime" , &fsTime, savePlot, 2);
    //pMap->addLogVar("countDelta50hz", &countDelta50hz, savePlot, 2);
    
    // IMU
    //pMap->addLogVar("IMU highDynamics" , &highDynamics, savePlot, 2);
    //pMap->addLogVar("IMU Acc X" , &pIMUdata->accel[0], savePlot, 2);
    //pMap->addLogVar("IMU Acc Y" , &pIMUdata->accel[1], savePlot, 2);
    //pMap->addLogVar("IMU Acc Z" , &pIMUdata->accel[2], savePlot, 2);
    
    //pMap->addLogVar("IMU dTheta X" , &dTheta_deg[0], savePlot, 2);
    //pMap->addLogVar("IMU dTheta Y" , &dTheta_deg[1], savePlot, 2);
    //pMap->addLogVar("IMU dTheta Z" , &dTheta_deg[2], savePlot, 2);
    //pMap->addLogVar("IMU dVel X" , &pIMUdata->dVelocity[0], savePlot, 2);
    //pMap->addLogVar("IMU dVel Y" , &pIMUdata->dVelocity[1], savePlot, 2);
    //pMap->addLogVar("IMU dVel Z" , &pIMUdata->dVelocity[2], savePlot, 2);
    
    //pMap->addLogVar("IMU Gyro X" , &pIMUdata->gyro[0], savePlot, 2);
    //pMap->addLogVar("IMU Gyro Y" , &pIMUdata->gyro[1], savePlot, 2);
    //pMap->addLogVar("IMU Gyro Z" , &pIMUdata->gyro[2], printSavePlot, 3);
    
    // Barometer
    //pMap->addLogVar("Baro Timestamp", &pBaroData->timestamp, savePlot, 2);
    //pMap->addLogVar("baro state", &baroState, savePlot, 2);
    //pMap->addLogVar("Baro Ref Pressure", &pBaroData->refPressure, savePlot, 2);
    //pMap->addLogVar("Baro Pressure", &pBaroData->pressure, savePlot, 2);
    //pMap->addLogVar("Baro pu", &pBaroData->pu, printSavePlot, 3);
    //pMap->addLogVar("Baro Temperature", &pBaroData->temperature, savePlot, 2);
    //pMap->addLogVar("Baro tu", &pBaroData->tu, savePlot, 2);
    //pMap->addLogVar("Baro Altitude", &pBaroData->altitude, savePlot, 2);
    
    // GPS
    //pMap->addLogVar("GPS Timestamp", &pGPSdata->timestamp, savePlot, 2);
    //pMap->addLogVar("GPS Time Since Startup", &pGPSdata->GPStimestamp, savePlot, 2);
    //pMap->addLogVar("FS - GPS Time", &pGPSdata->dt_GPS_FS, savePlot, 2);

    //pMap->addLogVar("GPS TTFF", &pGPSdata->ttff, savePlot, 2);
    //pMap->addLogVar("GPS Lat", &pGPSdata->posLLH[0], savePlot, 2);
    //pMap->addLogVar("GPS Lon", &pGPSdata->posLLH[1], savePlot, 2);
    //pMap->addLogVar("GPS Alt", &pGPSdata->posLLH[2], savePlot, 2);
    //pMap->addLogVar("GPS ref_posECEF[0]", &pGPSdata->ref_posECEF[0], savePlot, 2);
    //pMap->addLogVar("GPS ref_posECEF[1]", &pGPSdata->ref_posECEF[1], savePlot, 2);
    //pMap->addLogVar("GPS ref_posECEF[2]", &pGPSdata->ref_posECEF[2], savePlot, 2);
    //pMap->addLogVar("GPS posECEF[0]", &pGPSdata->posECEF[0], savePlot, 2);
    //pMap->addLogVar("GPS posECEF[1]", &pGPSdata->posECEF[1], savePlot, 2);
    //pMap->addLogVar("GPS posECEF[2]", &pGPSdata->posECEF[2], savePlot, 2);
    //pMap->addLogVar("GPS posENU[0]", &pGPSdata->posENU[0], savePlot, 2);
    //pMap->addLogVar("GPS posENU[1]", &pGPSdata->posENU[1], savePlot, 2);
    //pMap->addLogVar("GPS posENU[2]", &pGPSdata->posENU[2], savePlot, 2);
    //pMap->addLogVar("GPS 3D Fix Bias", &pGPSdata->alt_3dFixBias, savePlot, 2);
    //pMap->addLogVar("GPS horizPosAcc", &pGPSdata->horizPosAcc, savePlot, 2);
    //pMap->addLogVar("GPS vertPosAcc", &pGPSdata->vertPosAcc, savePlot, 2);
    //pMap->addLogVar("GPS VN", &pGPSdata->velNED[0], savePlot, 2);
    //pMap->addLogVar("GPS VE", &pGPSdata->velNED[1], savePlot, 2);
    //pMap->addLogVar("GPS VD", &pGPSdata->velNED[2], savePlot, 2);
    //pMap->addLogVar("GPS speedAcc", &pGPSdata->speedAcc, savePlot, 2);
    //pMap->addLogVar("GPS Fix", &gpsFix, savePlot, 2);
    //pMap->addLogVar("GPS Fix Ok", &fixOk, printSavePlot, 3);
    //pMap->addLogVar("GPS numSV", &numSV, savePlot, 2);
    
    //pMap->addLogVar("GPS eDOP", &pGPSdata->DOP.eDOP, savePlot, 2);
    //pMap->addLogVar("GPS gDOP", &pGPSdata->DOP.gDOP, savePlot, 2);
    //pMap->addLogVar("GPS hDOP", &pGPSdata->DOP.hDOP, savePlot, 2);
    //pMap->addLogVar("GPS nDOP", &pGPSdata->DOP.nDOP, savePlot, 2);
    //pMap->addLogVar("GPS pDOP", &pGPSdata->DOP.pDOP, savePlot, 2);
    //pMap->addLogVar("GPS tDOP", &pGPSdata->DOP.tDOP, savePlot, 2);
    //pMap->addLogVar("GPS vDOP", &pGPSdata->DOP.vDOP, savePlot, 2);
    //pMap->addLogVar("GPS timeSinceStartup", &pGPSdata->timeSinceStartup, savePlot, 2);
    
    //pMap->addLogVar("GPS Good", &gpsGood, savePlot, 2);
    //pMap->addLogVar("GPS rcvdByteCount", &gps_rcvdByteCount, printSavePlot, 3);
    //pMap->addLogVar("GPS rcvdMsgCount", &gps_rcvdMsgCount, printSavePlot, 3);
    //pMap->addLogVar("GPS Fifo writeByteCount", &gps_fifoWritedByteCount, printSavePlot, 3);
    
    // Navigation
    //pMap->addLogVar("Nav Pos N" , &pNavData->position[0], savePlot, 2);
    //pMap->addLogVar("Nav Pos E" , &pNavData->position[1], savePlot, 2);
    //pMap->addLogVar("Nav Alt", &pNavData->position[2], printSavePlot, 3);
    //pMap->addLogVar("Nav geoidCorrection", &pNavData->geoidCorrection, savePlot, 2);

    //pMap->addLogVar("Nav vel N" , &pNavData->velNED[0], savePlot, 2);
    //pMap->addLogVar("Nav vel E" , &pNavData->velNED[1], savePlot, 2);
    //pMap->addLogVar("Nav vel D" , &pNavData->velNED[2], savePlot, 2);

    //pMap->addLogVar("Nav velLL X" , &nav_velLL[0], savePlot, 2);
    //pMap->addLogVar("Nav velLL Y" , &nav_velLL[1], savePlot, 2);
    //pMap->addLogVar("Nav velLL Z" , &nav_velLL[2], savePlot, 2);
    
    //pMap->addLogVar("Nav dVelocity Z", &pNavData->stateInputs.dVelocity[2], savePlot, 2);
    //pMap->addLogVar("Nav Acc Z", &pNavData->accelBody[2], savePlot, 2);

    //pMap->addLogVar("Nav Roll Rate", &bodyRates_dps[0], savePlot, 2);
    //pMap->addLogVar("Nav Pitch Rate", &bodyRates_dps[1], savePlot, 2);
    //pMap->addLogVar("Nav Yaw Rate", &bodyRates_dps[2], savePlot, 2);
    
    pMap->addLogVar("Nav dTheta X", &nav_dTheta_deg[0], savePlot, 2);
    pMap->addLogVar("Nav dTheta Y", &nav_dTheta_deg[1], savePlot, 2);
    pMap->addLogVar("Nav dTheta Z", &nav_dTheta_deg[2], savePlot, 2);
    pMap->addLogVar("Nav dVel Z", &pNavData->stateInputs.dVelocity[2], savePlot, 2);
    
    
    //pMap->addLogVar("navstate", &navState, savePlot, 2);
    //pMap->addLogVar("INS Update Count", &NavUpdateCounts[INS], printSavePlot, 3);
    //pMap->addLogVar("Baro Update Count", &NavUpdateCounts[BaroUpdate], savePlot, 2);
    //pMap->addLogVar("GPS Update Count", &NavUpdateCounts[GPSUpdate], savePlot, 2);
    //pMap->addLogVar("Accel Update Count", &NavUpdateCounts[AccelUpdate], savePlot, 2);
    //pMap->addLogVar("Ground Align Update Count", &NavUpdateCounts[GroundAlign], savePlot, 2);
    
    //pMap->addLogVar("Baro Skipped Update Count", &NavSkippedUpdates[BaroUpdate], savePlot, 2);
    //pMap->addLogVar("GPS Skipped Update Count", &NavSkippedUpdates[GPSUpdate], savePlot, 2);
    //pMap->addLogVar("Accel Skipped Update Count", &NavSkippedUpdates[AccelUpdate], savePlot, 2);
    
    //pMap->addLogVar("Nav Timestamp", &pNavData->timestamp, savePlot, 2);
    //pMap->addLogVar("Nav IMU Timestamp", &pNavData->sensorTimestamp[INS], savePlot, 2);
    //pMap->addLogVar("Nav Baro Timestamp", &pNavData->sensorTimestamp[BaroUpdate], savePlot, 2);
    //pMap->addLogVar("Nav GPS Timestamp", &pNavData->sensorTimestamp[GPSUpdate], savePlot, 2);
    //pMap->addLogVar("Nav Accel Timestamp", &pNavData->sensorTimestamp[AccelUpdate], savePlot, 2);
    
    //pMap->addLogVar("Nav IMU dTimestamp", &pNavData->timestamp_diff[INS], savePlot, 2);
    //pMap->addLogVar("Nav Baro dTimestamp", &pNavData->timestamp_diff[BaroUpdate], savePlot, 2);
    //pMap->addLogVar("Nav GPS dTimestamp", &pNavData->timestamp_diff[GPSUpdate], savePlot, 2);
    //pMap->addLogVar("Nav Accel dTimestamp", &pNavData->timestamp_diff[AccelUpdate], savePlot, 2);
    
    //pMap->addLogVar("Nav dPitch", &pNavData->stateInputs.dTheta[1], savePlot, 2);
    //pMap->addLogVar("Nav Accel Mag" , &pNavData->accel_mag, savePlot, 2);
    //pMap->addLogVar("Nav Error Accel Roll" , &pNavError->accel_roll, savePlot, 2);
    //pMap->addLogVar("Nav Error Accel Pitch" , &pNavError->accel_pitch, savePlot, 2);

    //pMap->addLogVar("Nav Accel Roll" , &nav_accel_roll, printSavePlot, 3);
    //pMap->addLogVar("Nav Accel Pitch" , &nav_accel_pitch, printSavePlot, 3);
    
    //pMap->addLogVar("Nav Roll" , &eulerAnglesDeg[0], printSavePlot, 3);
    //pMap->addLogVar("Nav Pitch", &eulerAnglesDeg[1], printSavePlot, 3);
    //pMap->addLogVar("Nav Yaw"  , &eulerAnglesDeg[2], savePlot, 2);
    
    //pMap->addLogVar("Gyro Bias X (deg/s)" , &gyroBias_dps[0], savePlot, 2);
    //pMap->addLogVar("Gyro Bias Y (deg/s)" , &gyroBias_dps[1], savePlot, 2);
    //pMap->addLogVar("Gyro Bias Z (deg/s)" , &gyroBias_dps[2], savePlot, 2);
    //pMap->addLogVar("Accel Bias X (g)" , &accBias_g[0], savePlot, 2);
    //pMap->addLogVar("Accel Bias Y (g)" , &accBias_g[1], savePlot, 2);
    //pMap->addLogVar("Accel Bias Z (g)" , &accBias_g[2], savePlot, 2);
    //pMap->addLogVar("Gravity" , &pNavData->gravity, savePlot, 2);
    //pMap->addLogVar("Gravity Bias" , &pNavData->gravityBias, savePlot, 2);
    
    //pMap->addLogVar("q[0]", &pNavData->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("q[1]", &pNavData->q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("q[2]", &pNavData->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("q[3]", &pNavData->q_B_NED[3], savePlot, 2);
    
    //pMap->addLogVar("Roll Error", &pNavError->eulerAngles[0], savePlot, 2);
    //pMap->addLogVar("Pitch Error", &pNavError->eulerAngles[1], savePlot, 2);
    //pMap->addLogVar("Yaw Error", &pNavError->eulerAngles[2], savePlot, 2);
    
    //pMap->addLogVar("qError[0]", &pNavError->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("qError[1]", &pNavError->q_B_NED[1], printSavePlot, 3);
    //pMap->addLogVar("qError[2]", &pNavError->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("qError[3]", &pNavError->q_B_NED[3], savePlot, 2);
    
    //pMap->addLogVar("Vel Body X Error", &pNavError->velBody[0], savePlot, 2);
    //pMap->addLogVar("Vel Body Y Error", &pNavError->velBody[1], savePlot, 2);
    //pMap->addLogVar("Vel Body Z Error", &pNavError->velBody[2], savePlot, 2);
    
    //pMap->addLogVar("Vel N Error", &pNavError->velNED[0], savePlot, 2);
    //pMap->addLogVar("Vel E Error", &pNavError->velNED[1], savePlot, 2);
    //pMap->addLogVar("Vel D Error", &pNavError->velNED[2], savePlot, 2);
    
    //pMap->addLogVar("velLL X Error" , &nav_velLL_error[0], savePlot, 2);
    //pMap->addLogVar("velLL Y Error" , &nav_velLL_error[1], savePlot, 2);
    //pMap->addLogVar("velLL Z Error" , &nav_velLL_error[2], savePlot, 2);
    
    //pMap->addLogVar("N Error (m)", &pNavError->position[0], savePlot, 2);
    //pMap->addLogVar("E Error (m)", &pNavError->position[1], savePlot, 2);
    //pMap->addLogVar("Alt Error (m)", &altitudeError, savePlot, 2);
    
    //pMap->addLogVar("Acc Body X Error", &pNavError->accelBody[0], savePlot, 2);
    //pMap->addLogVar("Acc Body Y Error", &pNavError->accelBody[1], savePlot, 2);
    //pMap->addLogVar("Acc Body Z Error", &pNavError->accelBody[2], savePlot, 2);
    
    //pMap->addLogVar("Acc LL X Error", &nav_accelLL_error[0], savePlot, 2);
    //pMap->addLogVar("Acc LL Y Error", &nav_accelLL_error[1], savePlot, 2);
    //pMap->addLogVar("Acc LL Z Error", &nav_accelLL_error[2], savePlot, 2);
    
    //pMap->addLogVar("Acc Mag Error", &pNavError->accel_mag, savePlot, 2);
    
    //pMap->addLogVar("p Error", &pNavError->bodyRates[0], savePlot, 2);
    //pMap->addLogVar("q Error", &pNavError->bodyRates[1], savePlot, 2);
    //pMap->addLogVar("r Error", &pNavError->bodyRates[2], savePlot, 2);
    
    //pMap->addLogVar("Nav dTheta[0] Error", &pNavError->stateInputs.dTheta[0], savePlot, 2);
    //pMap->addLogVar("Nav dTheta[1] Error", &pNavError->stateInputs.dTheta[1], savePlot, 2);
    //pMap->addLogVar("Nav dTheta[2] Error", &pNavError->stateInputs.dTheta[2], savePlot, 2);
    //pMap->addLogVar("Nav dVelocity[2] Error", &pNavError->stateInputs.dVelocity[2], savePlot, 2);
    
    //pMap->addLogVar("P[ROLL][ROLL]"    , &Cov[ROLL][ROLL], savePlot, 2);
    //pMap->addLogVar("P[PITCH][PITCH]"    , &Cov[PITCH][PITCH], savePlot, 2);
    //pMap->addLogVar("P[ROLL][PITCH]"    , &Cov[ROLL][PITCH], savePlot, 2);
    //pMap->addLogVar("P[PITCH][ROLL]"    , &Cov[PITCH][ROLL], savePlot, 2);
    //pMap->addLogVar("P[YAW][YAW]"    , &Cov[YAW][YAW], savePlot, 2);
    //pMap->addLogVar("P[VN][VN]"    , &Cov[VN][VN], savePlot, 2);
    //pMap->addLogVar("P[VE][VE]"    , &Cov[VE][VE], savePlot, 2);
    //pMap->addLogVar("P[VD][VD]"    , &Cov[VD][VD], savePlot, 2);
    //pMap->addLogVar("P[VE][VE]"    , &Cov[VE][VE], savePlot, 2);
    //pMap->addLogVar("P[VD][VD]"    , &Cov[VD][VD], savePlot, 2);
    //pMap->addLogVar("P[VD][GRAVITY]"    , &Cov[VD][GRAVITY], savePlot, 2);
    //pMap->addLogVar("P[VD][GBIAS_X]"    , &Cov[VD][GBIAS_X], savePlot, 2);
    //pMap->addLogVar("P[VD][GBIAS_Y]"    , &Cov[VD][GBIAS_Y], savePlot, 2);
    //pMap->addLogVar("P[VD][ABIAS_Z]"    , &Cov[VD][ABIAS_Z], savePlot, 2);
    //pMap->addLogVar("P[N][N]"  , &Cov[N][N], savePlot, 2);
    //pMap->addLogVar("P[E][E]"  , &Cov[E][E], savePlot, 2);
    //pMap->addLogVar("P[ALT][ALT]"  , &Cov[ALT][ALT], printSavePlot, 3);
    //pMap->addLogVar("P[VD][ALT]"    , &Cov[VD][ALT], savePlot, 2);
    //pMap->addLogVar("P[ALT][VD]"    , &Cov[ALT][VD], savePlot, 2);
    //pMap->addLogVar("P[ALT][GRAVITY]"  , &Cov[ALT][GRAVITY], printSavePlot, 3);
    //pMap->addLogVar("P[ALT][GBIAS_X]"  , &Cov[ALT][GBIAS_X], printSavePlot, 3);
    //pMap->addLogVar("P[ALT][GBIAS_Y]"  , &Cov[ALT][GBIAS_Y], printSavePlot, 3);
    //pMap->addLogVar("P[ALT][ABIAS_Z]"  , &Cov[ALT][ABIAS_Z], printSavePlot, 3);
    //pMap->addLogVar("P[GBIAS_X][GBIAS_X]"    , &Cov[GBIAS_X][GBIAS_X], savePlot, 2);
    //pMap->addLogVar("P[GBIAS_Y][GBIAS_Y]"    , &Cov[GBIAS_Y][GBIAS_Y], savePlot, 2);
    //pMap->addLogVar("P[GBIAS_Z][GBIAS_Z]"    , &Cov[GBIAS_Z][GBIAS_Z], savePlot, 2);
    //pMap->addLogVar("P[ABIAS_X][ABIAS_X]"    , &Cov[ABIAS_X][ABIAS_X], savePlot, 2);
    //pMap->addLogVar("P[ABIAS_Y][ABIAS_Y]"    , &Cov[ABIAS_Y][ABIAS_Y], savePlot, 2);
    //pMap->addLogVar("P[ABIAS_Z][ABIAS_Z]"    , &Cov[ABIAS_Z][ABIAS_Z], savePlot, 2);
    //pMap->addLogVar("P[GRAVITY][GRAVITY]"    , &Cov[GRAVITY][GRAVITY], savePlot, 2);
    
    //pMap->addLogVar("P[VD][ALT]"    , &Cov[VD][ALT], savePlot, 2);
    //pMap->addLogVar("P[ALT][VD]"    , &Cov[ALT][VD], savePlot, 2);
    //pMap->addLogVar("P[ROLL][GBIAS_X]"    , &Cov[ROLL][GBIAS_X], savePlot, 2);
    //pMap->addLogVar("P[ROLL][ALT]"    , &Cov[ROLL][ALT], savePlot, 2);
    //pMap->addLogVar("P[PITCH][ALT]"    , &Cov[PITCH][ALT], printSavePlot, 3);
    //pMap->addLogVar("P[ROLL][N]"    , &Cov[ROLL][N], savePlot, 2);
    //pMap->addLogVar("P[N][ROLL]"    , &Cov[N][ROLL], savePlot, 2);
    //pMap->addLogVar("P[ROLL][E]"    , &Cov[ROLL][E], savePlot, 2);
    //pMap->addLogVar("P[E][ROLL]"    , &Cov[E][ROLL], savePlot, 2);
    //pMap->addLogVar("P[ROLL][ALT]"     , &Cov[ROLL][ALT], savePlot, 2);
    //pMap->addLogVar("P[ALT][ROLL]"     , &Cov[ALT][ROLL], savePlot, 2);
    
    //pMap->addLogVar("P[PITCH][N]"    , &Cov[PITCH][N], savePlot, 2);
    //pMap->addLogVar("P[N][PITCH]"    , &Cov[N][PITCH], savePlot, 2);
    //pMap->addLogVar("P[PITCH][E]"    , &Cov[PITCH][E], savePlot, 2);
    //pMap->addLogVar("P[E][PITCH]"    , &Cov[E][PITCH], savePlot, 2);
    //pMap->addLogVar("P[PITCH][ALT]"    , &Cov[PITCH][ALT], savePlot, 2);
    //pMap->addLogVar("P[ALT][PITCH]"    , &Cov[ALT][PITCH], savePlot, 2);
    
    //pMap->addLogVar("P[ROLL][VN]"    , &Cov[ROLL][VN], savePlot, 2);
    //pMap->addLogVar("P[VN][ROLL]"    , &Cov[VN][ROLL], savePlot, 2);
    //pMap->addLogVar("P[ROLL][VE]"    , &Cov[ROLL][VE], savePlot, 2);
    //pMap->addLogVar("P[VE][ROLL]"    , &Cov[VE][ROLL], savePlot, 2);
    //pMap->addLogVar("P[ROLL][VD]"     , &Cov[ROLL][VD], savePlot, 2);
    //pMap->addLogVar("P[VD][ROLL]"     , &Cov[VD][ROLL], savePlot, 2);
    
    //pMap->addLogVar("P[PITCH][VN]"    , &Cov[PITCH][VN], savePlot, 2);
    //pMap->addLogVar("P[VN][PITCH]"    , &Cov[VN][PITCH], savePlot, 2);
    //pMap->addLogVar("P[PITCH][VE]"    , &Cov[PITCH][VE], savePlot, 2);
    //pMap->addLogVar("P[VE][PITCH]"    , &Cov[VE][PITCH], savePlot, 2);
    //pMap->addLogVar("P[PITCH][VD]"    , &Cov[PITCH][VD], savePlot, 2);
    //pMap->addLogVar("P[VD][PITCH]"    , &Cov[VD][PITCH], savePlot, 2);
    //pMap->addLogVar("P[YAW][ALT]"    , &Cov[YAW][ALT], savePlot, 2);
    //pMap->addLogVar("P[GRAVITY][ALT]"    , &Cov[GRAVITY][ALT], savePlot, 2);
    //pMap->addLogVar("P[GRAVITY][VD]"    , &Cov[GRAVITY][VD], savePlot, 2);
    
    //pMap->addLogVar("Roll Stdev"    , &navStd[ROLL][ROLL], savePlot, 2);
    //pMap->addLogVar("Roll 3 Stdev"  , &nav3Std[ROLL][ROLL], savePlot, 2);
    //pMap->addLogVar("Pitch Stdev"   , &navStd[PITCH][PITCH], savePlot, 2);
    //pMap->addLogVar("Pitch Stdev Due to Alt"   , &navStd[PITCH][ALT], savePlot, 2);
    //pMap->addLogVar("Roll Stdev Due to Alt"   , &navStd[ROLL][ALT], savePlot, 2);
    //pMap->addLogVar("Yaw Stdev Due to VN"   , &navStd[YAW][VN], savePlot, 2);
    //pMap->addLogVar("Yaw Stdev Due to VE"   , &navStd[YAW][VE], savePlot, 2);
    //pMap->addLogVar("Pitch 3 Stdev" , &nav3Std[PITCH][PITCH], savePlot, 2);
    //pMap->addLogVar("Yaw Stdev"   , &navStd[YAW][YAW], savePlot, 2);
    //pMap->addLogVar("N Stdev"    , &navStd[N][N], savePlot, 2);
    //pMap->addLogVar("E Stdev"    , &navStd[E][E], savePlot, 2);
    //pMap->addLogVar("Alt Stdev"    , &navStd[ALT][ALT], savePlot, 2);
    //pMap->addLogVar("VN Stdev"    , &navStd[VN][VN], savePlot, 2);
    //pMap->addLogVar("VE Stdev"    , &navStd[VE][VE], savePlot, 2);
    //pMap->addLogVar("VD Stdev"    , &navStd[VD][VD], savePlot, 2);
    //pMap->addLogVar("Accel Bias X Stdev"    , &navStd[ABIAS_X][ABIAS_X], savePlot, 2);
    //pMap->addLogVar("Accel Bias Y Stdev"    , &navStd[ABIAS_Y][ABIAS_Y], savePlot, 2);
    //pMap->addLogVar("Accel Bias Z Stdev"    , &navStd[ABIAS_Z][ABIAS_Z], savePlot, 2);
    //pMap->addLogVar("VN AX Stdev"    , &navStd[VN][ABIAS_X], savePlot, 2);
    //pMap->addLogVar("AX VN Stdev"    , &navStd[ABIAS_X][VN], savePlot, 2);
    //pMap->addLogVar("VE AX Stdev"    , &navStd[VE][ABIAS_X], savePlot, 2);
    //pMap->addLogVar("covCorrection[PITCH][ALT]", &covarianceCorrection[PITCH][ALT], savePlot, 2);
    //pMap->addLogVar("covCorrection[ROLL][ALT]", &covarianceCorrection[ROLL][ALT], savePlot, 2);
    //pMap->addLogVar("covCorrection[VD][ALT]", &covarianceCorrection[VD][ALT], savePlot, 2);
    //pMap->addLogVar("covCorrection[ALT][ALT]", &covarianceCorrection[ALT][ALT], savePlot, 2);
    
    //pMap->addLogVar("Accel Roll Stdev" , &stdAccel[ACCEL_ROLL][ACCEL_ROLL], savePlot, 2);
    //pMap->addLogVar("Accel Pitch Stdev", &stdAccel[ACCEL_PITCH][ACCEL_PITCH], savePlot, 2);
    
    //pMap->addLogVar("Q[Q0][Q0]"    , &PN[Q0][Q0], savePlot, 2);
    //pMap->addLogVar("Q[Q1][Q1]"    , &PN[Q1][Q1], savePlot, 2);
    //pMap->addLogVar("Q[Q2][Q2]"    , &PN[Q2][Q2], savePlot, 2);
    //pMap->addLogVar("Q[Q3][Q3]"    , &PN[Q3][Q3], savePlot, 2);
    //pMap->addLogVar("Q[VN][VN]"    , &PN[VN][VN], savePlot, 2);
    //pMap->addLogVar("Q[VE][VE]"    , &PN[VE][VE], savePlot, 2);
    //pMap->addLogVar("Q[VD][VD]"    , &PN[VD][VD], savePlot, 2);
    //pMap->addLogVar("Q[LAT][LAT]"  , &PN[LAT][LAT], savePlot, 2);
    //pMap->addLogVar("Q[LON][LON]"  , &PN[LON][LON], savePlot, 2);
    //pMap->addLogVar("Q[ALT][ALT]"  , &PN[ALT][ALT], savePlot, 2);
    
    //pMap->addLogVar("KB[ROLL][BARO_ALT]"    , &KB[ROLL][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[PITCH][BARO_ALT]"    , &KB[PITCH][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[YAW][BARO_ALT]"    , &KB[YAW][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[VN][BARO_ALT]"    , &KB[VN][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[VE][BARO_ALT]"    , &KB[VE][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[VD][BARO_ALT]"    , &KB[VD][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[LAT][BARO_ALT]"  , &KB[LAT][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[LON][BARO_ALT]"  , &KB[LON][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[ALT][BARO_ALT]"  , &KB[ALT][BARO_ALT], savePlot, 2);
    
    //pMap->addLogVar("baroResidual[BAR_ALT]", &barometerResidual[BARO_ALT], savePlot, 2);
    
    //pMap->addLogVar("GPSResidual[GPS_N]", &GPSResidual[GPS_N], savePlot, 2);
    //pMap->addLogVar("GPSResidual[GPS_E]", &GPSResidual[GPS_E], savePlot, 2);
    //pMap->addLogVar("GPSResidual[GPS_ALT]", &GPSResidual[GPS_ALT], savePlot, 2);
    //pMap->addLogVar("GPSResidual[GPS_VN]", &GPSResidual[GPS_VN], savePlot, 2);
    //pMap->addLogVar("GPSResidual[GPS_VE]", &GPSResidual[GPS_VE], savePlot, 2);
    //pMap->addLogVar("GPSResidual[GPS_VD]", &GPSResidual[GPS_VD], savePlot, 2);
    
    //pMap->addLogVar("GPSFilterError[GPS_N]", &GPSFilterError[GPS_N], savePlot, 2);
    //pMap->addLogVar("GPSFilterError[GPS_E]", &GPSFilterError[GPS_E], savePlot, 2);
    //pMap->addLogVar("GPSFilterError[GPS_ALT]", &GPSFilterError[GPS_ALT], savePlot, 2);
    //pMap->addLogVar("GPSFilterError[GPS_VN]", &GPSFilterError[GPS_VN], savePlot, 2);
    //pMap->addLogVar("GPSFilterError[GPS_VE]", &GPSFilterError[GPS_VE], savePlot, 2);
    //pMap->addLogVar("GPSFilterError[GPS_VD]", &GPSFilterError[GPS_VD], savePlot, 2);
    
    //pMap->addLogVar("GroundResiudal[GROUND_N]", &FsNavigation_getGroundResidual()[GROUND_N], savePlot, 2);
    //pMap->addLogVar("GroundResiudal[GROUND_E]", &FsNavigation_getGroundResidual()[GROUND_E], savePlot, 2);
    //pMap->addLogVar("GroundResiudal[GROUND_ALT]", &FsNavigation_getGroundResidual()[GROUND_ALT], savePlot, 2);
    //pMap->addLogVar("GroundResiudal[GROUND_VN]", &FsNavigation_getGroundResidual()[GROUND_VN], savePlot, 2);
    //pMap->addLogVar("GroundResiudal[GROUND_VE]", &FsNavigation_getGroundResidual()[GROUND_VE], savePlot, 2);
    //pMap->addLogVar("GroundResiudal[GROUND_VD]", &FsNavigation_getGroundResidual()[GROUND_VD], savePlot, 2);
    
    //pMap->addLogVar("stdGPS[GPS_N][GPS_N]"    , &stdGPS[GPS_N][GPS_N], savePlot, 2);
    //pMap->addLogVar("stdGPS[GPS_E][GPS_E]"    , &stdGPS[GPS_E][GPS_E], savePlot, 2);
    //pMap->addLogVar("stdGPS[GPS_ALT][GPS_ALT]", &stdGPS[GPS_ALT][GPS_ALT], savePlot, 2);
    //pMap->addLogVar("stdGPS[GPS_VN][GPS_VN]"  , &stdGPS[GPS_VN][GPS_VN], savePlot, 2);
    //pMap->addLogVar("stdGPS[GPS_VE][GPS_VE]"  , &stdGPS[GPS_VE][GPS_VE], savePlot, 2);
    //pMap->addLogVar("stdGPS[GPS_VD][GPS_VD]"  , &stdGPS[GPS_VD][GPS_VD], savePlot, 2);
    
    // Controls
    //pMap->addLogVar("control mode", &ctrlMode, savePlot, 2);
    
    //pMap->addLogVar("dT"          , &pControlData->dT     , savePlot, 2);
    //pMap->addLogVar("dTmin"       , &pControlData->dTmin  , savePlot, 2);
    //pMap->addLogVar("dTmax"       , &pControlData->dTmax  , savePlot, 2);
    //pMap->addLogVar("da"          , &pControlData->da     , savePlot, 2);
    //pMap->addLogVar("de"          , &pControlData->de     , savePlot, 2);
    //pMap->addLogVar("dr"          , &pControlData->dr     , savePlot, 2);
    //pMap->addLogVar("daRaw"       , &pControlData->daRaw  , savePlot, 2);
    //pMap->addLogVar("deRaw"       , &pControlData->deRaw  , savePlot, 2);
    //pMap->addLogVar("drRaw"       , &pControlData->drRaw  , savePlot, 2);
    //pMap->addLogVar("minCtrl"     , &pControlData->minCtrl, savePlot, 2);
    //pMap->addLogVar("maxCtrl"     , &pControlData->maxCtrl, savePlot, 2);
    
    //pMap->addLogVar("daMomentCmd", &daMomentCmd, savePlot, 2);
    //pMap->addLogVar("deMomentCmd", &deMomentCmd, savePlot, 2);
    //pMap->addLogVar("drMomentCmd", &drMomentCmd, savePlot, 2);
    
    pMap->addLogVar("Ctrl PMW [0]", &pControlData->TPWM[0], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [1]", &pControlData->TPWM[1], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [2]", &pControlData->TPWM[2], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [3]", &pControlData->TPWM[3], savePlot, 2);
    
    //pMap->addLogVar("Ctrl rollCmd", &rollCmd, savePlot, 2);
    //pMap->addLogVar("Ctrl pitchCmd", &pitchCmd, savePlot, 2);
    //pMap->addLogVar("Ctrl Vx int err", &pControlData->vx_int_error, savePlot, 2);
    //pMap->addLogVar("Ctrl Vy int err", &pControlData->vy_int_error, savePlot, 2);
    //pMap->addLogVar("Ctrl Vz int err", &pControlData->vz_int_error, savePlot, 2);
    //pMap->addLogVar("Ctrl yawRateCmd", &yawRateCmd, savePlot, 2);
    //pMap->addLogVar("Ctrl VLLxCmd", &pControlData->VLLxCmd, savePlot, 2);
    //pMap->addLogVar("Ctrl VLLyCmd", &pControlData->VLLyCmd, savePlot, 2);
    //pMap->addLogVar("Ctrl VLLzCmd", &pControlData->VLLzCmd, printSavePlot, 3);
    //pMap->addLogVar("Ctrl hCmd", &pControlData->hCmd, savePlot, 2);
    //pMap->addLogVar("controlAltitude", &controlAltitude, savePlot, 2);
    //pMap->addLogVar("takeOff", &takeOff, printSavePlot, 3);
    //pMap->addLogVar("crashLand", &crashLand, savePlot, 2);
    //pMap->addLogVar("onGround", &onGround, savePlot, 2);
    //pMap->addLogVar("movingDetection", &movingDetection, savePlot, 2);
    
    // Thrust Estimator
    //pMap->addLogVar("Thrust Est [0]", &pThrustEstData->motorThrustEst[0], savePlot, 2);
    //pMap->addLogVar("Thrust Est [1]", &pThrustEstData->motorThrustEst[1], savePlot, 2);
    //pMap->addLogVar("Thrust Est [2]", &pThrustEstData->motorThrustEst[2], savePlot, 2);
    //pMap->addLogVar("Thrust Est [3]", &pThrustEstData->motorThrustEst[3], savePlot, 2);
    pMap->addLogVar("RPM Est [0]", &pThrustEstData->motorRPM[0], savePlot, 2);
    pMap->addLogVar("RPM Est [1]", &pThrustEstData->motorRPM[1], savePlot, 2);
    pMap->addLogVar("RPM Est [2]", &pThrustEstData->motorRPM[2], savePlot, 2);
    pMap->addLogVar("RPM Est [3]", &pThrustEstData->motorRPM[3], savePlot, 2);
    //pMap->addLogVar("Motor Moment Est [0]", &pThrustEstData->bodyMomentEst[0], savePlot, 2);
    //pMap->addLogVar("Motor Moment Est [1]", &pThrustEstData->bodyMomentEst[1], savePlot, 2);
    //pMap->addLogVar("Motor Moment Est [2]", &pThrustEstData->bodyMomentEst[2], savePlot, 2);
    //pMap->addLogVar("Total Thrust Est", &pThrustEstData->bodyThrustEst[2], savePlot, 2);
    //pMap->addLogVar("Motor Rate Est [0]", &pThrustEstData->bodyRate[0], savePlot, 2);
    //pMap->addLogVar("Motor Rate Est [1]", &pThrustEstData->bodyRate[1], savePlot, 2);
    //pMap->addLogVar("Motor Rate Est [2]", &pThrustEstData->bodyRate[2], savePlot, 2);
    //pMap->addLogVar("Motor Ang Accel Est [0]", &pThrustEstData->bodyAngAccel[0], savePlot, 2);
    //pMap->addLogVar("Motor Ang Accel Est [1]", &pThrustEstData->bodyAngAccel[1], savePlot, 2);
    //pMap->addLogVar("Motor Ang Accel Est [2]", &pThrustEstData->bodyAngAccel[2], savePlot, 2);
    //pMap->addLogVar("Motor az", &pThrustEstData->az, savePlot, 2);

    pMap->addLogVar("Thrust Error [0]", &pThrustError->motorThrustEst[0], savePlot, 2);
    pMap->addLogVar("Thrust Error [1]", &pThrustError->motorThrustEst[1], savePlot, 2);
    pMap->addLogVar("Thrust Error [2]", &pThrustError->motorThrustEst[2], savePlot, 2);
    pMap->addLogVar("Thrust Error [3]", &pThrustError->motorThrustEst[3], savePlot, 2);
    pMap->addLogVar("RPM Error [0]", &pThrustError->motorRPM[0], savePlot, 2);
    pMap->addLogVar("RPM Error [1]", &pThrustError->motorRPM[1], savePlot, 2);
    pMap->addLogVar("RPM Error [2]", &pThrustError->motorRPM[2], savePlot, 2);
    pMap->addLogVar("RPM Error [3]", &pThrustError->motorRPM[3], savePlot, 2);
    pMap->addLogVar("Motor Moment Error [0]", &pThrustError->bodyMomentEst[0], savePlot, 2);
    pMap->addLogVar("Motor Moment Error [1]", &pThrustError->bodyMomentEst[1], savePlot, 2);
    pMap->addLogVar("Motor Moment Error [2]", &pThrustError->bodyMomentEst[2], savePlot, 2);
    pMap->addLogVar("Total Thrust Error", &pThrustError->bodyThrustEst[2], savePlot, 2);
    
    pMap->addLogVar("Motor Rate Est Error [0]", &pThrustError->bodyRate[0], savePlot, 2);
    pMap->addLogVar("Motor Rate Est Error [1]", &pThrustError->bodyRate[1], savePlot, 2);
    pMap->addLogVar("Motor Rate Est Error [2]", &pThrustError->bodyRate[2], savePlot, 2);
    pMap->addLogVar("Motor Ang Accel Est Error [0]", &pThrustError->bodyAngAccel[0], savePlot, 2);
    pMap->addLogVar("Motor Ang Accel Est Error [1]", &pThrustError->bodyAngAccel[1], savePlot, 2);
    pMap->addLogVar("Motor Ang Accel Est Error [2]", &pThrustError->bodyAngAccel[2], savePlot, 2);
    pMap->addLogVar("Motor az Error", &pThrustError->az, savePlot, 2);
    
    pMap->addLogVar("Motor deltaAngle[0] Error", &pThrustError->deltaAngle[0], savePlot, 2);
    pMap->addLogVar("Motor deltaAngle[1] Error", &pThrustError->deltaAngle[1], savePlot, 2);
    pMap->addLogVar("Motor deltaAngle[2] Error", &pThrustError->deltaAngle[2], savePlot, 2);
    pMap->addLogVar("Motor deltaVz Error", &pThrustError->deltaVz, savePlot, 2);
    
    //pMap->addLogVar("Motor dTheta X", &motor_dTheta_deg[0], savePlot, 2);
    //pMap->addLogVar("Motor dTheta Y", &motor_dTheta_deg[1], savePlot, 2);
    //pMap->addLogVar("Motor dTheta Z", &motor_dTheta_deg[2], savePlot, 2);
    //pMap->addLogVar("Motor dVel Z", &pThrustEstData->deltaVz, savePlot, 2);
    
    pMap->addLogVar("Motor dTheta[0] Residual", &pThrustEstData->imuResidual[0], savePlot, 2);
    pMap->addLogVar("Motor dTheta[1] Residual", &pThrustEstData->imuResidual[1], savePlot, 2);
    pMap->addLogVar("Motor dTheta[2] Residual", &pThrustEstData->imuResidual[2], savePlot, 2);
    pMap->addLogVar("Motor vZ Residual", &pThrustEstData->imuResidual[3], savePlot, 2);
}
#endif

// **********************************************************************
// Initialization
// **********************************************************************
void initializeRoutines(void)
{
    // Initialize Time Routines
    for (int i=0; i<nRoutines; i++)
    {
        performRoutine[i] = false;
        routineDelays[i] = 0.0;
        actualDelays[i]  = 0.0;
    }
    
    // Initialize Event Rountines
    for (int i=0; i<nEvents; i++)
    {
        eventStartTimes[i] = 0.0;
        eventStarted[i] = false;
    }
}

void initializeTime(void)
{
    for (int i=0; i<nRoutines; i++)
    {
        prevTime[i] = getTime();
    }
    
    for (int i=0; i<nEvents; i++)
    {
        eventStartTimes[i] += getTime();
    }
}

void setupIO(void)
{
    // Serial
    Serial.begin(baudRate);
    Wire.begin(); // 18 SDA, 19 SCL
    display("Setup Begin.\n");
    
    // LED Pin
    pinMode(LEDPIN, OUTPUT);
    
    // IMU
    FsImu_setupIMU(accSensitivity, gyroSensitivity);
    
    // Barometer
    FsBarometer_setupBarometer();
    FsBarometer_setPressureResolution(bmp180Resolution);
    
    // Navigation
    FsNavigation_setupNavigation(initialPosition, initialHeading, loadIMUCalibration);
#ifdef SIMULATION
    if (useTruthStateTran) { FsNavigation_setTruthTransitionMatrix(); }
#endif
    // GPS
    FsGPS_setupGPS(gpsBaudRate, fs_allow2DFix);
    
    // PWM In
    FsPwmIn_setup();
    
    // Controls
    FsControls_setup();
    FsControls_setMode(controlMode);
    
    // Thrust Estimator
    FsThrustEstimator_setup();
    
    // Telemetry
    FsTelemetry_setupTelemetry(TMBaudRate, TM_MSG_Rates);
    FsTelemetry_setTimingPointer(actualDelays);
    if (configTM)
    {
        FsTelemetry_configureTelemetry();
    }
    
    LEDoff();
    display("Setup Complete.\n");
}

void controlsModing()
{
    double vn_variance;
    double vn_std;
    double ve_variance;
    double ve_std;
    bool velEstimateGood;
    
    // Determing if velocity estimate is within bounds
    vn_variance = fabs( *(FsNavigation_getCovariance() + VN*NSTATES + VN) );
    ve_variance = fabs( *(FsNavigation_getCovariance() + VE*NSTATES + VE) );
    
    vn_std = sqrt(vn_variance);
    ve_std = sqrt(ve_variance);
    
    velEstimateGood = (vn_std < VELSTDLIM) & (ve_std < VELSTDLIM);
    
    // Mode controls
    if (pControlData->mode == AttitudeControl || pControlData->mode == VelocityControl)
    {
        if (pControlData->mode == AttitudeControl && velEstimateGood)
        {
            controlMode = VelocityControl;
            FsControls_setMode(controlMode);
        }
        else if (pControlData->mode == VelocityControl && !velEstimateGood)
        {
            controlMode = AttitudeControl;
            FsControls_setMode(controlMode);
        }
    }
    
}

void performSerialIO()
{
    FsCommon_performSerialIO();
    FsGPS_performSerialIO();
    FsTelemetry_performSerialIO();
}

void printData()
{
#ifdef PRINT
    bool anyPrint = false;
    
    if (printTiming)
    {
        display(getTime());
        display(" ");
        
        display("1hz rate: ");
        display( 1.0/actualDelays[hz1] );
        display(", ");
        
        display("50hz rate: ");
        display( 1.0/actualDelays[hz50] );
        display(", ");
        
        display("100hz rate: ");
        display( 1.0/actualDelays[hz100] );
        display(", ");
        
        display("200hz rate: ");
        display( 1.0/actualDelays[hz200] );
        display(", ");
        
        display("600hz rate: ");
        display( 1.0/actualDelays[hz600] );
        display("\n");
        
        anyPrint = true;
    }
    
    if (printIMU)
    {
        display(getTime());
        display(" ");
        
        display("IMU I2C code: ");
        display(pIMUdata->errorCodeIMU);
        display(" ");
        
        display("gyro (deg/s): ");
        display(pIMUdata->gyro[0] * radian2degree);
        display(" ");
        display(pIMUdata->gyro[1] * radian2degree);
        display(" ");
        display(pIMUdata->gyro[2] * radian2degree);
        display(", ");
        
        display("accel (g): ");
        display(pIMUdata->accel[0] / (Gravity));
        display(" ");
        display(pIMUdata->accel[1] / (Gravity));
        display(" ");
        display(pIMUdata->accel[2] / (Gravity));
        display("\n");
        
        anyPrint = true;
    }
    
    if (printCalibration && pNavData->state != Calibration)
    {
        display(getTime());
        display(" ");
        
        display("gyro bias (deg/s): ");
        display(pNavData->gyroBias[0]*radian2degree);
        display(" ");
        display(pNavData->gyroBias[1]*radian2degree);
        display(" ");
        display(pNavData->gyroBias[2]*radian2degree);
        display(", ");
        
        display("accel bias (g): ");
        display(pNavData->accBias[0]/Gravity);
        display(" ");
        display(pNavData->accBias[1]/Gravity);
        display(" ");
        display(pNavData->accBias[2]/Gravity);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printCalibratedIMU)
    {
        display(getTime());
        display(" ");
        
        display("gyro (deg/s): ");
        display((pIMUdata->gyro[0] - pNavData->gyroBias[0])*radian2degree);
        display(" ");
        display((pIMUdata->gyro[1] - pNavData->gyroBias[1])*radian2degree);
        display(" ");
        display((pIMUdata->gyro[2] - pNavData->gyroBias[2])*radian2degree);
        display(", ");
        
        display("accel (m/s/s): ");
        display(pIMUdata->accel[0] - pNavData->accBias[0]);
        display(" ");
        display(pIMUdata->accel[1] - pNavData->accBias[1]);
        display(" ");
        display(pIMUdata->accel[2] - pNavData->accBias[2]);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printBarometer)
    {
        display(getTime());
        display(" ");
        
        display("Baro I2C code: ");
        display(pBaroData->errorCodeBaro);
        display("\n");
        
        display("Baro timestamp/pressure/temperature/altitude: ");
        display(pBaroData->timestamp);
        display(" ");
        display(pBaroData->pressure);
        display(" ");
        display(pBaroData->temperature);
        display(" ");
        display(pBaroData->altitude);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printGPS)
    {
        display(getTime());
        display(" ");
        
        display("GPS timestamp: ");
        display(pGPSdata->timestamp);
        display("\n");
        
        display("GPStimestamp/dt_FS_GPS/TTFF: ");
        display(pGPSdata->GPStimestamp);
        display(" ");
        display(pGPSdata->dt_GPS_FS);
        display(" ");
        display(pGPSdata->ttff);
        display("\n");
        
        display("GPS FIFO rcvdCount/writeCount: ");
        display((int) pGPSdata->fifoReadCount);
        display(" ");
        display((int) pGPSdata->fifoWriteCount);
        display("\n");
        
        display("GPS FIFO maxRead/maxWrite: ");
        display((int) pGPSdata->fifoMaxReadLength);
        display(" ");
        display((int) pGPSdata->fifoMaxWriteLength);
        display("\n");
        
        display("GPS rcvdByte/rcvdMsg: ");
        display((int) pGPSdata->rcvdByteCount);
        display(" ");
        display((int) pGPSdata->rcvdMsgCount);
        display("\n");
        
        display("GPS Fix/Fix Ok/numSV: ");
        display((int) pGPSdata->gpsFix);
        display(",");
        display(pGPSdata->fixOk);
        display(",");
        display(pGPSdata->numSV);
        display("\n");
        
        display("GPS posLLH: ");
        display(" [");
        display(pGPSdata->posLLH[0]); display(","); display(pGPSdata->posLLH[1]); display(","); display(pGPSdata->posLLH[2]);
        display("]\n");
        
        display("GPS posNEU: ");
        display(" [");
        display(pGPSdata->posENU[1]); display(","); display(pGPSdata->posENU[0]); display(","); display(pGPSdata->posENU[2]);
        display("]\n");
        
        display("GPS velNED: ");
        display(" [");
        display(pGPSdata->velNED[0]); display(","); display(pGPSdata->velNED[1]); display(","); display(pGPSdata->velNED[2]);
        display("]\n");
        
        display("GPS hor/vert/speed Acc: ");
        display(pGPSdata->horizPosAcc);
        display(" ");
        display(pGPSdata->vertPosAcc);
        display(" ");
        display(pGPSdata->speedAcc);
        display(" ");
        display("\n");
        
        anyPrint = true;
    }
    
    if (printGPS_DOP)
    {
        display(getTime());
        display(" ");
        
        display("GPS timestamp: ");
        display(pGPSdata->timestamp);
        display("\n");
        
        display("GPS DOP: [g,p,t,v,n,e]");
        display(" [");
        display(pGPSdata->DOP.gDOP); display(","); display(pGPSdata->DOP.pDOP); display(","); display(pGPSdata->DOP.tDOP); display(",");
        display(pGPSdata->DOP.vDOP); display(","); display(pGPSdata->DOP.nDOP); display(","); display(pGPSdata->DOP.eDOP);
        display("]\n");
        
        anyPrint = true;
    }
    
    if (printNavInputs)
    {
        display(getTime());
        display(" ");
        
        display("Body Rates (deg/s): ");
        display(pNavData->bodyRates[0] * radian2degree);
        display(" ");
        display(pNavData->bodyRates[1] * radian2degree);
        display(" ");
        display(pNavData->bodyRates[2] * radian2degree);
        display(", ");
        
        display("Body Accel (m/s/s): ");
        display(pNavData->accelBody[0]);
        display(" ");
        display(pNavData->accelBody[1]);
        display(" ");
        display(pNavData->accelBody[2]);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printAngles)
    {
        display(getTime());
        display(" ");
        
        display("Attitude (deg): ");
        display(pNavData->eulerAngles[0]*radian2degree);
        display(" ");
        display(pNavData->eulerAngles[1]*radian2degree);
        display(" ");
        display(pNavData->eulerAngles[2]*radian2degree);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printVelocity)
    {
        display(getTime());
        display(" ");
        
        display("velNED (m/s): ");
        display(pNavData->velNED[0]);
        display(" ");
        display(pNavData->velNED[1]);
        display(" ");
        display(pNavData->velNED[2]);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printPosition)
    {
        display(getTime());
        display(" ");
        
        display("N/E/ALT (m/m/m): ");
        display(pNavData->position[0]);
        display(" ");
        display(pNavData->position[1]);
        display(" ");
        display(pNavData->position[2]);
        display("\n");
        
        anyPrint = true;
    }

    if (printNavVariance)
    {
        display(getTime());
        display(" ");
        display("Attitude Std (deg): ");
        display(sqrt(FsNavigation_getCovariance()[ROLL*NSTATES+ROLL])*radian2degree);
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[PITCH*NSTATES+PITCH])*radian2degree);
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[YAW*NSTATES+YAW])*radian2degree);
        display("\n");
        
        display(getTime());
        display(" ");
        display("velNED Std (m/s): ");
        display(sqrt(FsNavigation_getCovariance()[VN*NSTATES+VN]));
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[VE*NSTATES+VE]));
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[VD*NSTATES+VD]));
        display("\n");
        
        display(getTime());
        display(" ");
        display("N/E/ALT Std (m/m/m): ");
        display(sqrt(FsNavigation_getCovariance()[N*NSTATES+N]));
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[E*NSTATES+E]));
        display(" ");
        display(sqrt(FsNavigation_getCovariance()[ALT*NSTATES+ALT]));
        display("\n");
    }
    
    if (printNavFilterCount)
    {
        display(getTime());
        display(" ");
        
        display("[INS, Baro, GPS, Accel, Ground] Filter Count: [");
        display(pNavData->updateCount[INS]);
        display(",");
        display(pNavData->updateCount[BaroUpdate]);
        display(",");
        display(pNavData->updateCount[GPSUpdate]);
        display(",");
        display(pNavData->updateCount[AccelUpdate]);
        display(",");
        display(pNavData->updateCount[GroundAlign]);
        display("]\n");
        
        anyPrint = true;
    }
    
    if (printAccelFilter)
    {
        display(getTime());
        display(" ");
        
        display("[accel_roll, accel_pitch, accel_mag, accel_count]: [");
        display(pNavData->accel_roll * radian2degree);
        display(" ");
        display(pNavData->accel_pitch * radian2degree);
        display(" ");
        display(pNavData->accel_mag);
        display(" ");
        display(pNavData->updateCount[AccelUpdate]);
        display("]\n");
        
        anyPrint = true;
    }
    
    if (printPWMIn)
    {
        display(getTime());
        display(" ");
        
        display("pwmIn: ");
        display(pControlData->pwmCmd[THROTTLE_CHANNEL]);
        display(" ");
        display(pControlData->pwmCmd[ROLL_CHANNEL]);
        display(" ");
        display(pControlData->pwmCmd[PITCH_CHANNEL]);
        display(" ");
        display(pControlData->pwmCmd[YAW_CHANNEL]);
        display("\n");
    }
    
    if (printCommands)
    {
        display(getTime());
        display(" ");
        
        display("velLL Cmds (m/s): ");
        display(pControlData->VLLxCmd);
        display(" ");
        display(pControlData->VLLyCmd);
        display(" ");
        display(pControlData->VLLzCmd);
        display(", ");
        
        display("Attitude Cmds (deg): ");
        display(pControlData->rollCmd*radian2degree);
        display(" ");
        display(pControlData->pitchCmd*radian2degree);
        display(" ");
        display(pControlData->yawRateCmd*radian2degree);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printMotorPWM)
    {
        display(getTime());
        display(" ");
        
        display("Motor PWM: ");
        display(pControlData->TPWM[0]);
        display(" ");
        display(pControlData->TPWM[1]);
        display(" ");
        display(pControlData->TPWM[2]);
        display(" ");
        display(pControlData->TPWM[3]);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printTakeOff)
    {
        display("[takeOff, onGround, movingDetection, crashLand]: [");
        display((int) pControlData->takeOff);
        display(", ");
        display((int) pControlData->onGround);
        display(", ");
        display((int) pControlData->movingDetection);
        display(", ");
        display((int) pControlData->crashLand);
        display("]\n");
        
        anyPrint = true;
    }
    
    if (printTelemetry)
    {
        display(getTime());
        display(" ");

        display("TM startTime: [");
        for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
        {
            display(pTMData->tm_start_time[tm_id]);
            if (tm_id < N_TM_MSGS-1) { display(", "); }
            else { display("];\n"); }
        }
        
        display("TM writeMsgCount: [");
        for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
        {
            display(pTMData->tm_write_msg_count[tm_id]);
            if (tm_id < N_TM_MSGS-1) { display(", "); }
            else { display("];\n"); }
        }
        
        display("TM maxRate: [");
        for (unsigned int tm_id = FS_TM_IMU; tm_id < N_TM_MSGS; tm_id++)
        {
            display(pTMData->max_tm_rate[tm_id]);
            if (tm_id < N_TM_MSGS-1) { display(", "); }
            else { display("];\n"); }
        }
        
        display("TM rcvdByte/writeByte/maxWriteByte: ");
        display((int) pTMData->tm_rcv_byte_count);
        display(" ");
        display((int) pTMData->tm_write_byte_count);
        display(" ");
        display((int) pTMData->tm_max_write_byte_count);
        display("\n");
        
        display("TM timeout/failedWrite/fifoFull: ");
        display((int) pTMData->tm_timeout_count);
        display(" ");
        display((int) pTMData->tm_failed_write_count);
        display(" ");
        display((int) pTMData->tm_write_fifo_full_count);
        display("\n");
        
        FsTelemetry_resetMaxWriteCounter();
        
        anyPrint = true;
    }
    
    if (anyPrint) { display("\n"); }
#endif
}
