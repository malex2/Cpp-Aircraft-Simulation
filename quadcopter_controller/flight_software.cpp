//
//  flight_software.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "flight_software.hpp"
#include "fs_common.hpp"
#include "fs_imu.hpp"
#include "fs_pwmin.hpp"
#include "fs_controls.hpp"
#include "fs_navigation.hpp"

#ifdef SIMULATION
#include "dynamics_model.hpp"
#endif

// General Settings
long baudRate;
bool intiailized  = false;
#ifdef SIMULATION
ArduinoSerial Serial;
#endif

// Simulation Classes
class ModelMap* pMap  = 0;

// Time keeping (s)
enum {hz50, hz100, hz200, hz800, printRoutine, nRoutines};
double prevTime[nRoutines];
double routineDelays[nRoutines];
bool   performRoutine[nRoutines];
double actualDelays[nRoutines];

// Event keeping (s)
enum {startGNC, imuWarmup, nEvents};
double eventStartTimes[nEvents];
bool eventStarted[nEvents];

// IMU
IMUtype* pIMUdata = 0;
accSensitivityType accSensitivity;
gyroSensitivityType gyroSensitivity;

// Navigation
bool useTruthNav;
double initialPosition[3];
NavType* pNavData = 0;
#ifdef SIMULATION
NavType* pNavError = 0;
#endif

// Controls
ControlType* pControlData = 0;
ControlMode controlMode;

// Print
double position[3];
double eulerAnglesDeg[3];
double eulerRatesDps[3];
double bodyRatesDps[3];
double dVelIMU[3];
double dThetaIMU[3];

// Errors
double positionError[3];
double velocityError[3];
double attitudeError[3];
double attitdeCmdError[3];
double eulerRateError[3];
double bodyRateError[3];
double accelBodyError[3];

// **********************************************************************
// Initialize
// **********************************************************************
void initialize(void)
{
    // Setup
    initializeVariables();
    
    // General Settings
    baudRate = 9600; // 9600, 38400, 115200
    
    // IMU Settings
    accSensitivity  = accSensitivity_4g;
    gyroSensitivity = gyroSensitivity_250dps;
    
    // Navigation Settings
    initialPosition[0] = 28.5997222;  // Latitude  (deg)
    initialPosition[1] = -81.3394444; // Longitude (deg)
    initialPosition[2] = 0.6;         // Altitude  (m)
    useTruthNav = true;
    
    // Control Settings
    controlMode = ThrottleControl;
    
    // Timing Settings
    routineDelays[hz50]  = 1.0/50.0;
    routineDelays[hz100] = 1.0/100.0;
    routineDelays[hz200] = 1.0/200.0;
    routineDelays[hz800] = 1.0/800.0;
    routineDelays[printRoutine] = 0.5;
    
    // Event Settings
    eventStartTimes[imuWarmup] = 1.0;
    
    // Finish Setup
    getModels();
    setup();
    
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
        performRoutine[i] = getTime() - prevTime[i] >= routineDelays[i];
    }

    // 800 houtine
    if (performRoutine[hz800])
    {
        // Timing info
        actualDelays[hz800] = getTime() - prevTime[hz800];
        prevTime[hz800] = getTime();
        
        // Update IMU
        FsImu_performIMU();
        pIMUdata = FsImu_getIMUdata();
        
        // Update IMU Print Variables
        for (int i=0; i<3; i++)
        {
            dVelIMU[i] = pIMUdata->dVelocity[i];
            dThetaIMU[i] = pIMUdata->dTheta[i];
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
        FsNavigation_setIMUdata(pIMUdata);
        FsNavigation_calibrateIMU();
    }
    
    if (performRoutine[hz100])
    {
        actualDelays[hz100] = getTime() - prevTime[hz100];
        prevTime[hz100] = getTime();
    }
    
    // 50 hz routine
    if (performRoutine[hz50])
    {
        actualDelays[hz50] = getTime() - prevTime[hz50];
        prevTime[hz50] = getTime();

        // Update Inertial Navigation
        FsNavigation_performNavigation();
        pNavData = FsNavigation_getNavData(useTruthNav);
        
        // Read Controller PWM
        FsPwmIn_performPwmIn();
        
        // Perform Guidance
        
        // Set commands
        FsControls_setThrottleCmd   ( FsPwmIn_getValues()[THROTTLE] );
        FsControls_setRollCmd       ( FsPwmIn_getValues()[ROLL]     );
        FsControls_setPitchCmd      ( FsPwmIn_getValues()[PITCH]    );
        FsControls_setBodyYawRateCmd( FsPwmIn_getValues()[YAWRATE]  );
        
        // Perform Attitde Control
        FsControls_setIMUdata(pIMUdata);
        FsControls_setNavdata(pNavData);
        FsControls_performControls();
        
        pControlData = FsControls_getControlData();
        
        FsImu_zeroDelta(true);
    }

    // Print Variables
    pNavError = FsNavigation_getNavError();
    for (int i=0; i<3; i++)
    {
        if (i != 3) { position[i] = pNavData->position[i]/deg2rad; }
        eulerAnglesDeg[i] = pNavData->eulerAngles[i]/deg2rad;
        eulerRatesDps[i]  = pNavData->eulerRates[i]/deg2rad;
        bodyRatesDps[i]   = pNavData->bodyRates[i]/deg2rad;
    }
    
    return true;
}

// **********************************************************************
// Initialization Functions
// **********************************************************************
void getModels()
{
    
    // Get Pointers
    pIMUdata = FsImu_getIMUdata();
    pNavData = FsNavigation_getNavData();
    pNavError = FsNavigation_getNavError();
    pControlData = FsControls_getControlData();
    
# ifdef SIMULATION
    if(pMap)
    {
        // Set Simulation Pointers
        FsCommon_setSimulationModels(pMap);
        FsNavigation_setSimulationModels(pMap);
        FsControls_setSimulationModels(pMap);
        FsImu_setSimulationModels(pMap);
        
        setPrintVariables();
    }
    else
    {
        display("Warning: Could not find map models.");
        display("\n");
    }
#endif
}

void flightSoftware_setMapPointer(ModelMap* pMapInit)
{
#ifdef SIMULATION
    pMap = pMapInit;
#endif
}

void setPrintVariables()
{
#ifdef SIMULATION
    // IMU
    /*
    pMap->addLogVar("IMU Acc X" , &pIMUdata->accel[0], savePlot, 2);
    pMap->addLogVar("IMU Acc Y" , &pIMUdata->accel[1], savePlot, 2);
    pMap->addLogVar("IMU Acc Z" , &pIMUdata->accel[2], savePlot, 2);
    
    pMap->addLogVar("IMU Gyro X" , &pIMUdata->gyro[0], savePlot, 2);
    pMap->addLogVar("IMU Gyro Y" , &pIMUdata->gyro[1], savePlot, 2);
    pMap->addLogVar("IMU Gyro Z" , &pIMUdata->gyro[2], savePlot, 2);
    
    pMap->addLogVar("IMU dVel X" , &dVelIMU[0], savePlot, 2);
    pMap->addLogVar("IMU dVel Y" , &dVelIMU[1], savePlot, 2);
    pMap->addLogVar("IMU dVel Z" , &dVelIMU[2], savePlot, 2);
    
    pMap->addLogVar("IMU dTheta X" , &dThetaIMU[0], savePlot, 2);
    pMap->addLogVar("IMU dTheta Y" , &dThetaIMU[1], savePlot, 2);
    pMap->addLogVar("IMU dTheta Z" , &dThetaIMU[2], savePlot, 2);
    
    // Navigation
    pMap->addLogVar("Nav Lat" , &position[0], savePlot, 2);
    pMap->addLogVar("Nav Lon" , &position[1], savePlot, 2);
    pMap->addLogVar("Nav wgs84 Alt", &position[2], savePlot, 2);
    pMap->addLogVar("Nav MSL Alt"  , &pNavData->mslAlt, savePlot, 2);
    
    pMap->addLogVar("Nav Gravity Body X" , &pNavData->gravityBody[0], savePlot, 2);
    pMap->addLogVar("Nav Gravity Body Y" , &pNavData->gravityBody[1], savePlot, 2);
    pMap->addLogVar("Nav Gravity Body Z" , &pNavData->gravityBody[2], savePlot, 2);
    
    pMap->addLogVar("Nav Acc Body X" , &pNavData->accelBody[0], savePlot, 2);
    pMap->addLogVar("Nav Acc Body Y" , &pNavData->accelBody[1], savePlot, 2);
    pMap->addLogVar("Nav Acc Body Z" , &pNavData->accelBody[2], savePlot, 2);
    
    pMap->addLogVar("Nav vel N" , &pNavData->velNED[0], savePlot, 2);
    pMap->addLogVar("Nav vel E" , &pNavData->velNED[1], savePlot, 2);
    pMap->addLogVar("Nav vel D" , &pNavData->velNED[2], savePlot, 2);
    */
    pMap->addLogVar("Nav Roll" , &eulerAnglesDeg[0], savePlot, 2);
    pMap->addLogVar("Nav Pitch", &eulerAnglesDeg[1], savePlot, 2);
    pMap->addLogVar("Nav Yaw"  , &eulerAnglesDeg[2], savePlot, 2);
    
    pMap->addLogVar("Nav Roll Rate" , &eulerRatesDps[0], savePlot, 2);
    pMap->addLogVar("Nav Pitch Rate", &eulerAnglesDeg[1], savePlot, 2);
    pMap->addLogVar("Nav Yaw Rate"  , &eulerAnglesDeg[2], savePlot, 2);
    
    pMap->addLogVar("Nav p", &bodyRatesDps[0], savePlot, 2);
    pMap->addLogVar("Nav q", &bodyRatesDps[1], savePlot, 2);
    pMap->addLogVar("Nav r", &bodyRatesDps[2], savePlot, 2);
    
    pMap->addLogVar("Roll Error", &pNavError->eulerAngles[0], savePlot, 2);
    pMap->addLogVar("Pitch Error", &pNavError->eulerAngles[1], savePlot, 2);
    pMap->addLogVar("Yaw Error", &pNavError->eulerAngles[2], savePlot, 2);
    
    //pMap->addLogVar("Roll Rate Error X", &pNavError->eulerRates[0], savePlot, 2);
    //pMap->addLogVar("Pitch Rate Error Y", &pNavError->eulerRates[1], savePlot, 2);
    //pMap->addLogVar("Yaw Rate Error", &pNavError->eulerRates[2], savePlot, 2);
    
    //pMap->addLogVar("Body Rate X Error", &pNavError->bodyRates[0], savePlot, 2);
    //pMap->addLogVar("Body Rate Y Error", &pNavError->bodyRates[1], savePlot, 2);
    //pMap->addLogVar("Body Rate Z Error", &pNavError->bodyRates[2], savePlot, 2);
#endif
}

// **********************************************************************
// Initialization
// **********************************************************************
void initializeVariables(void)
{
    // Initialize Time Routines
    for (int i=0; i<nRoutines; i++)
    {
        prevTime[i] = 0.0;
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

void setup(void)
{
    // Serial
    Serial.begin(baudRate);
    display("Setup Begin.\n");
    
    // LED Pin
    pinMode(LEDPIN, OUTPUT);
    
    // IMU
    FsImu_setupIMU(accSensitivity, gyroSensitivity);
    
    // Navigation
    FsNavigation_setupNavigation(initialPosition);
    
    // PWM In
    FsPwmIn_setup();
    
    // Controls
    FsControls_setup();
    FsControls_setMode(controlMode);
    
    LEDoff();
    display("Setup Complete.\n");
}
