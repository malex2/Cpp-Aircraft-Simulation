//
//  flight_software.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "flight_software.hpp"
#include "fs_imu.hpp"
#include "fs_pwmin.hpp"
#include "fs_controls.hpp"
#include "fs_navigation.hpp"

#ifdef SIMULATION
    #include "dynamics_model.hpp"
#endif

// General Settings
int baudRate;
bool intiailized  = false;

// Simulation Classes
#ifdef SIMULATION
    class ModelMap* pMap = 0;
    ArduinoSerial Serial;
#endif

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
const double dtPad = 1e-10;

// IMU
IMUtype* pIMUdata = 0;
accSensitivityType accSensitivity;
gyroSensitivityType gyroSensitivity;

// Navigation
bool useTruthNav;
double initialPosition[3];
double initialHeading;
NavType* pNavData = 0;
#ifdef SIMULATION
    NavType* pNavError = 0;
#endif

// Controls
ControlType* pControlData = 0;
ControlMode controlMode;

// Print
#ifdef SIMULATION
double position[3];
double eulerAnglesDeg[3];
double fsTime;
double countDelta50hz;
double navState = 0.0;
double altitudeError = 0.0;
double cpwmCmd;
#endif

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
    accSensitivity  = accSensitivity_8g;
    gyroSensitivity = gyroSensitivity_250dps;
    
    // Navigation Settings
    initialPosition[0] = 28.5997222;  // Latitude  (deg)
    initialPosition[1] = -81.3394444; // Longitude (deg)
    initialPosition[2] = 0.6;         // Altitude  (m)
    initialHeading = 0.0;
    useTruthNav = false;
    
    // Control Settings
    controlMode = AttitudeControl;
    
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
    setupIO();
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

    // 800 houtine
    if (performRoutine[hz800])
    {
        // Timing info
        actualDelays[hz800] = getTime() - prevTime[hz800];
        prevTime[hz800] = getTime();
        
        // Update IMU
        FsImu_performIMU( actualDelays[hz800] );
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
        
        // Update Inertial Navigation
        FsNavigation_performNavigation( actualDelays[hz100] );
        
        // Perform Attitde Control
        FsControls_setControlsData(pIMUdata, pNavData);
        FsControls_performControls();
    }
    
    // 50 hz routine
    if (performRoutine[hz50])
    {
        actualDelays[hz50] = getTime() - prevTime[hz50];
        prevTime[hz50] = getTime();
        
        // Read Controller PWM
        FsPwmIn_performPwmIn();
        
        // Perform Guidance
  
        // Set commands
        FsControls_setPWMCommands( FsPwmIn_getPWM() );
    }
#ifdef SIMULATION
    // Print Variables
    fsTime = getTime();
    
    navState = static_cast<double> (pNavData->state);
    
    for (int i=0; i<3; i++)
    {
        if (i != 3) { position[i] = pNavData->position[i]*radian2degree; }
        eulerAnglesDeg[i] = pNavData->eulerAngles[i]*radian2degree;
    }

    pNavError = FsNavigation_getNavError();
    altitudeError = -pNavError->position[2];
    cpwmCmd = getPwmCmd(THROTTLE);
#endif
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
    pControlData = FsControls_getControlData();
    
# ifdef SIMULATION
    pNavError = FsNavigation_getNavError();
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
    // Timing
    //pMap->addLogVar("fsTime" , &fsTime, savePlot, 2);
    //pMap->addLogVar("countDelta50hz", &countDelta50hz, savePlot, 2);
    
    // IMU
    pMap->addLogVar("IMU Acc X" , &pIMUdata->accel[0], savePlot, 2);
    pMap->addLogVar("IMU Acc Y" , &pIMUdata->accel[1], savePlot, 2);
    pMap->addLogVar("IMU Acc Z" , &pIMUdata->accel[2], savePlot, 2);
    
    pMap->addLogVar("IMU Gyro X" , &pIMUdata->gyro[0], savePlot, 2);
    pMap->addLogVar("IMU Gyro Y" , &pIMUdata->gyro[1], savePlot, 2);
    pMap->addLogVar("IMU Gyro Z" , &pIMUdata->gyro[2], savePlot, 2);
    
    // Navigation
    //pMap->addLogVar("Nav Lat" , &position[0], savePlot, 2);
    //pMap->addLogVar("Nav Lon" , &position[1], savePlot, 2);
    //pMap->addLogVar("Nav wgs84 Alt", &pNavData->position[2], printSavePlot, 3);
    
    //pMap->addLogVar("Nav vel N" , &pNavData->velNED[0], savePlot, 2);
    //pMap->addLogVar("Nav vel E" , &pNavData->velNED[1], savePlot, 2);
    //pMap->addLogVar("Nav vel D" , &pNavData->velNED[2], savePlot, 2);

    //pMap->addLogVar("navstate", &navState, savePlot, 2);
    pMap->addLogVar("Nav Roll" , &eulerAnglesDeg[0], printSavePlot, 3);
    pMap->addLogVar("Nav Pitch", &eulerAnglesDeg[1], printSavePlot, 3);
    pMap->addLogVar("Nav Yaw"  , &eulerAnglesDeg[2], savePlot, 2);
    
    //pMap->addLogVar("q[0]", &pNavData->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("q[1]", &pNavData->q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("q[2]", &pNavData->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("q[3]", &pNavData->q_B_NED[3], savePlot, 2);
    
    pMap->addLogVar("Roll Error", &pNavError->eulerAngles[0], savePlot, 2);
    pMap->addLogVar("Pitch Error", &pNavError->eulerAngles[1], savePlot, 2);
    pMap->addLogVar("Yaw Error", &pNavError->eulerAngles[2], savePlot, 2);
    
    //pMap->addLogVar("qError[0]", &pNavError->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("qError[1]", &pNavError->q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("qError[2]", &pNavError->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("qError[3]", &pNavError->q_B_NED[3], savePlot, 2);
    
    pMap->addLogVar("Vel Body X Error", &pNavError->velBody[0], savePlot, 2);
    pMap->addLogVar("Vel Body Y Error", &pNavError->velBody[1], savePlot, 2);
    pMap->addLogVar("Vel Body Z Error", &pNavError->velBody[2], savePlot, 2);
    
    pMap->addLogVar("Vel N Error", &pNavError->velNED[0], savePlot, 2);
    pMap->addLogVar("Vel E Error", &pNavError->velNED[1], savePlot, 2);
    pMap->addLogVar("Vel D Error", &pNavError->velNED[2], savePlot, 2);
    
    //pMap->addLogVar("Lat Error (deg)", &pNavError->position[0], savePlot, 2);
    //pMap->addLogVar("Lon Error (deg)", &pNavError->position[1], savePlot, 2);
    pMap->addLogVar("Alt Err (m)"    , &altitudeError, savePlot, 2);
    
    //pMap->addLogVar("Acc Body X Error", &pNavError->accelBody[0], savePlot, 2);
    //pMap->addLogVar("Acc Body Y Error", &pNavError->accelBody[1], savePlot, 2);
    //pMap->addLogVar("Acc Body Z Error", &pNavError->accelBody[2], savePlot, 2);
    
    // Controls
    pMap->addLogVar("da"          , &pControlData->da     , savePlot, 2);
    pMap->addLogVar("de"          , &pControlData->de     , savePlot, 2);
    pMap->addLogVar("dr"          , &pControlData->dr     , savePlot, 2);
    pMap->addLogVar("Ctrl PMW [0]", &pControlData->TPWM[0], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [1]", &pControlData->TPWM[1], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [2]", &pControlData->TPWM[2], savePlot, 2);
    pMap->addLogVar("Ctrl PMW [3]", &pControlData->TPWM[3], savePlot, 2);
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

void setupIO(void)
{
    // Serial
    Serial.begin(baudRate);
    display("Setup Begin.\n");
    
    // LED Pin
    pinMode(LEDPIN, OUTPUT);
    
    // IMU
    FsImu_setupIMU(accSensitivity, gyroSensitivity);
    
    // Navigation
    FsNavigation_setupNavigation(initialPosition, initialHeading);
    
    // PWM In
    FsPwmIn_setup();
    
    // Controls
    FsControls_setup();
    FsControls_setMode(controlMode);
    
    LEDoff();
    display("Setup Complete.\n");
}
