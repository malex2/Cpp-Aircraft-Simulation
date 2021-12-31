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

//#define PRINT

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
bool printAngles;
bool printVelocity;
bool printPosition;
bool printPWMIn;
bool printCommands;
bool printMotorPWM;
#endif
// Simulation Classes
#ifdef SIMULATION
    class ModelMap* pMap = 0;
    ArduinoSerial Serial;
#endif

// Time keeping (s)
enum {hz1, hz50, hz100, hz200, hz800, printRoutine, nRoutines};
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

// Barometer
barometerType* pBaroData = 0;
byte bmp180Resolution;
bool useBarometer;

// GPS
bool useGPS;

// Navigation
bool useTruthNav;
double initialPosition[3];
double initialHeading;
NavType* pNavData = 0;
#ifdef SIMULATION
    NavType* pNavError = 0;
#endif
#ifdef FILTERTEST
NavType* pStateError = 0;
NavType* pLinStateError = 0;
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
    double baroState = 0.0;

    double Cov[NSTATES][NSTATES];
    double PN[NSTATES][NSTATES];
    double KB[NSTATES][NBAROSTATES];
    double filterStateError[NSTATES];
#endif

// **********************************************************************
// Initialize
// **********************************************************************
void initialize(void)
{
    // Setup
    initializeVariables();
    
    // Print Settings
#ifdef PRINT
    printTiming        = false;
    printIMU           = true;
    printCalibration   = false;
    printCalibratedIMU = false;
    printBarometer     = true;
    printAngles        = false;
    printVelocity      = false;
    printPosition      = false;
    printPWMIn         = false;
    printCommands      = false;
    printMotorPWM      = false;
#endif
    
    // General Settings
    baudRate = 9600; // 9600, 38400, 115200
    
    // IMU Settings
    accSensitivity  = accSensitivity_4g;
    gyroSensitivity = gyroSensitivity_500dps;
    
    // Barometer Settings
    bmp180Resolution = BMP180_COMMAND_PRESSURE3;
    useBarometer     = true;
    
    // Navigation Settings
    initialPosition[0] = 28.5997222;  // Latitude  (deg)
    initialPosition[1] = -81.3394444; // Longitude (deg)
    initialPosition[2] = 0.6;         // Altitude  (m)
    initialHeading = 0.0;
    useTruthNav = false;
    
    // GPS
    useGPS = false;
    
    // Control Settings
    controlMode = AttitudeControl; //NoControl, ThrottleControl, AttitudeControl;
    
    // Timing Settings
    routineDelays[hz1]   = 1.0;
    routineDelays[hz50]  = 1.0/50.0;
    routineDelays[hz100] = 1.0/100.0;//1.0/100.0;
    routineDelays[hz200] = 1.0/200.0;//1.0/200.0;
    routineDelays[hz800] = 1.0/800.0;//1.0/800.0;
    routineDelays[printRoutine] = 1.0;
    
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
    }
    
    if (performRoutine[hz1])
    {
        //Timing Info
        actualDelays[hz1] = getTime() - prevTime[hz1];
        prevTime[hz1] = getTime();
        
        bool goodStart = FsBarometer_startBarometerMeasurement();
        if (!goodStart) { display("Barometer not ready\n"); }
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
    
    navState = static_cast<double> (pNavData->state);
    
    baroState = static_cast<double> (pBaroData->state);
    
    for (int i=0; i<3; i++)
    {
        if (i != 3) { position[i] = pNavData->position[i]*radian2degree; }
        eulerAnglesDeg[i] = pNavData->eulerAngles[i]*radian2degree;
    }

    pNavError = FsNavigation_getNavError();
    altitudeError = -pNavError->position[2];
    cpwmCmd = (double) pControlData->pwmCmd[THROTTLE];
    
    double* Ptemp = FsNavigation_getCovariance();
    double* Qtemp = FsNavigation_getProcessNoise();
    double* Ktemp = FsNavigation_getBaroKalmanGain();
    double* stemp = FsNavigation_getStateError();
    for (int i=0; i<NSTATES; i++)
    {
        filterStateError[i] = *(stemp+i);
        for (int j=0; j<NSTATES; j++)
        {
            if (j < NBAROSTATES){ KB[i][j] = *(Ktemp+i*NBAROSTATES+j); }
            PN[i][j]  = *(Qtemp+i*NSTATES+j);
            Cov[i][j] = *(Ptemp+i*NSTATES+j);
        }
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
    pNavData  = FsNavigation_getNavData();
    pControlData = FsControls_getControlData();
    
    FsNavigation_setIMUdata(pIMUdata);
    
#ifdef FILTERTEST
    pStateError = FsNavigation_getNavStateError();
    pLinStateError = FsNavigation_getLinStateError();
#endif
    
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
    //pMap->addLogVar("IMU Acc X" , &pIMUdata->accel[0], savePlot, 2);
    //pMap->addLogVar("IMU Acc Y" , &pIMUdata->accel[1], savePlot, 2);
    pMap->addLogVar("IMU Acc Z" , &pIMUdata->accel[2], savePlot, 2);
    
    //pMap->addLogVar("IMU Gyro X" , &pIMUdata->gyro[0], savePlot, 2);
    //pMap->addLogVar("IMU Gyro Y" , &pIMUdata->gyro[1], savePlot, 2);
    //pMap->addLogVar("IMU Gyro Z" , &pIMUdata->gyro[2], savePlot, 2);
    
    // Barometer
    pMap->addLogVar("Baro Timestamp", &pBaroData->timestamp, savePlot, 2);
    //pMap->addLogVar("baro state", &baroState, savePlot, 2);
    //pMap->addLogVar("Baro Pressure", &pBaroData->pressure, savePlot, 2);
    //pMap->addLogVar("Baro pu", &pBaroData->pu, printSavePlot, 3);
    //pMap->addLogVar("Baro Temperature", &pBaroData->temperature, savePlot, 2);
    //pMap->addLogVar("Baro tu", &pBaroData->tu, savePlot, 2);
    pMap->addLogVar("Baro Altitude", &pBaroData->altitude, savePlot, 2);
    
    // Navigation
    //pMap->addLogVar("Nav Lat" , &position[0], savePlot, 2);
    //pMap->addLogVar("Nav Lon" , &position[1], savePlot, 2);
    pMap->addLogVar("Nav wgs84 Alt", &pNavData->position[2], printSavePlot, 3);
    
    //pMap->addLogVar("Nav vel N" , &pNavData->velNED[0], savePlot, 2);
    //pMap->addLogVar("Nav vel E" , &pNavData->velNED[1], savePlot, 2);
    pMap->addLogVar("Nav vel D" , &pNavData->velNED[2], savePlot, 2);

    //pMap->addLogVar("navstate", &navState, savePlot, 2);
    //pMap->addLogVar("Nav Roll" , &eulerAnglesDeg[0], savePlot, 2);
    //pMap->addLogVar("Nav Pitch", &eulerAnglesDeg[1], savePlot, 2);
    //pMap->addLogVar("Nav Yaw"  , &eulerAnglesDeg[2], savePlot, 2);
    
    //pMap->addLogVar("q[0]", &pNavData->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("q[1]", &pNavData->q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("q[2]", &pNavData->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("q[3]", &pNavData->q_B_NED[3], savePlot, 2);
    
    //pMap->addLogVar("Roll Error", &pNavError->eulerAngles[0], savePlot, 2);
    //pMap->addLogVar("Pitch Error", &pNavError->eulerAngles[1], savePlot, 2);
    //pMap->addLogVar("Yaw Error", &pNavError->eulerAngles[2], savePlot, 2);
    
    //pMap->addLogVar("qError[0]", &pNavError->q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("qError[1]", &pNavError->q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("qError[2]", &pNavError->q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("qError[3]", &pNavError->q_B_NED[3], savePlot, 2);
    
    //pMap->addLogVar("Vel Body X Error", &pNavError->velBody[0], savePlot, 2);
    //pMap->addLogVar("Vel Body Y Error", &pNavError->velBody[1], savePlot, 2);
    //pMap->addLogVar("Vel Body Z Error", &pNavError->velBody[2], savePlot, 2);
    
    //pMap->addLogVar("Vel N Error", &pNavError->velNED[0], savePlot, 2);
    //pMap->addLogVar("Vel E Error", &pNavError->velNED[1], savePlot, 2);
    pMap->addLogVar("Vel D Error", &pNavError->velNED[2], savePlot, 2);
    
    //pMap->addLogVar("Lat Error (deg)", &pNavError->position[0], savePlot, 2);
    //pMap->addLogVar("Lon Error (deg)", &pNavError->position[1], savePlot, 2);
    pMap->addLogVar("Alt Err (m)"    , &altitudeError, savePlot, 2);
    
    //pMap->addLogVar("Acc Body X Error", &pNavError->accelBody[0], savePlot, 2);
    //pMap->addLogVar("Acc Body Y Error", &pNavError->accelBody[1], savePlot, 2);
    //pMap->addLogVar("Acc Body Z Error", &pNavError->accelBody[2], savePlot, 2);
#ifdef FILTERTEST
    /*
    pMap->addLogVar("q0 Lin State Error", &pLinStateError->q_B_NED[0], savePlot, 2);
    pMap->addLogVar("q1 Lin State Error", &pLinStateError->q_B_NED[1], savePlot, 2);
    pMap->addLogVar("q2 Lin State Error", &pLinStateError->q_B_NED[2], savePlot, 2);
    pMap->addLogVar("q3 Lin State Error", &pLinStateError->q_B_NED[3], savePlot, 2);
    pMap->addLogVar("Roll Lin State Error (deg)", &pLinStateError->eulerAngles[0], savePlot, 2);
    pMap->addLogVar("Pitch Lin State Error (deg)", &pLinStateError->eulerAngles[1], savePlot, 2);
    pMap->addLogVar("Yaw Lin State Error (deg)", &pLinStateError->eulerAngles[2], savePlot, 2);
    pMap->addLogVar("Vel N Lin State Error", &pLinStateError->velNED[0], savePlot, 2);
    pMap->addLogVar("Vel E Lin State Error", &pLinStateError->velNED[1], savePlot, 2);
    pMap->addLogVar("Vel D Lin State Error", &pLinStateError->velNED[2], savePlot, 2);
    pMap->addLogVar("Lat Lin State Error (deg)", &pLinStateError->position[0], savePlot, 2);
    pMap->addLogVar("Lon Lin State Error (deg)", &pLinStateError->position[1], savePlot, 2);
    pMap->addLogVar("Alt Lin State Err (m)"    , &pLinStateError->position[2], savePlot, 2);
    */
    
    //pMap->addLogVar("P[Q0][Q0]"    , &Cov[Q0][Q0], savePlot, 2);
    //pMap->addLogVar("P[Q1][Q1]"    , &Cov[Q1][Q1], savePlot, 2);
    //pMap->addLogVar("P[Q2][Q2]"    , &Cov[Q2][Q2], savePlot, 2);
    //pMap->addLogVar("P[Q3][Q3]"    , &Cov[Q3][Q3], savePlot, 2);
    //pMap->addLogVar("P[VN][VN]"    , &Cov[VN][VN], savePlot, 2);
    //pMap->addLogVar("P[VE][VE]"    , &Cov[VE][VE], savePlot, 2);
    pMap->addLogVar("P[VD][VD]"    , &Cov[VD][VD], savePlot, 2);
    //pMap->addLogVar("P[LAT][LAT]"  , &Cov[LAT][LAT], savePlot, 2);
    //pMap->addLogVar("P[LON][LON]"  , &Cov[LON][LON], savePlot, 2);
    pMap->addLogVar("P[ALT][ALT]"  , &Cov[ALT][ALT], savePlot, 2);
    
    //pMap->addLogVar("Q[Q0][Q0]"    , &PN[Q0][Q0], savePlot, 2);
    //pMap->addLogVar("Q[Q1][Q1]"    , &PN[Q1][Q1], savePlot, 2);
    //pMap->addLogVar("Q[Q2][Q2]"    , &PN[Q2][Q2], savePlot, 2);
    //pMap->addLogVar("Q[Q3][Q3]"    , &PN[Q3][Q3], savePlot, 2);
    //pMap->addLogVar("Q[VN][VN]"    , &PN[VN][VN], savePlot, 2);
    //pMap->addLogVar("Q[VE][VE]"    , &PN[VE][VE], savePlot, 2);
    //pMap->addLogVar("Q[VD][VD]"    , &PN[VD][VD], savePlot, 2);
    //pMap->addLogVar("Q[LAT][LAT]"  , &PN[LAT][LAT], savePlot, 2);
    //pMap->addLogVar("Q[LON][LON]"  , &PN[LON][LON], savePlot, 2);
    pMap->addLogVar("Q[ALT][ALT]"  , &PN[ALT][ALT], savePlot, 2);
    
    
    //pMap->addLogVar("KB[Q0][BARO_ALT]"    , &KB[Q0][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[Q1][BARO_ALT]"    , &KB[Q1][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[Q2][BARO_ALT]"    , &KB[Q2][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[Q3][BARO_ALT]"    , &KB[Q3][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[VN][BARO_ALT]"    , &KB[VN][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[VE][BARO_ALT]"    , &KB[VE][BARO_ALT], savePlot, 2);
    pMap->addLogVar("KB[VD][BARO_ALT]"    , &KB[VD][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[LAT][BARO_ALT]"  , &KB[LAT][BARO_ALT], savePlot, 2);
    //pMap->addLogVar("KB[LON][BARO_ALT]"  , &KB[LON][BARO_ALT], savePlot, 2);
    pMap->addLogVar("KB[ALT][BARO_ALT]"  , &KB[ALT][BARO_ALT], savePlot, 2);
    
    //pMap->addLogVar("stateError[Q0]"  , &filterStateError[Q0], savePlot, 2);
    //pMap->addLogVar("stateError[Q1]"  , &filterStateError[Q1], savePlot, 2);
    //pMap->addLogVar("stateError[Q2]"  , &filterStateError[Q2], savePlot, 2);
    //pMap->addLogVar("stateError[Q3]"  , &filterStateError[Q3], savePlot, 2);
    //pMap->addLogVar("stateError[VN]"  , &filterStateError[VN], savePlot, 2);
    //pMap->addLogVar("stateError[VE]"  , &filterStateError[VE], savePlot, 2);
    pMap->addLogVar("stateError[VD]"  , &filterStateError[VD], savePlot, 2);
    //pMap->addLogVar("stateError[LAT]" , &filterStateError[LAT], savePlot, 2);
    //pMap->addLogVar("stateError[LON]" , &filterStateError[LON], savePlot, 2);
    pMap->addLogVar("stateError[ALT]" , &filterStateError[ALT], savePlot, 2);
    
    /*
    pMap->addLogVar("q0 State Error", &pStateError->q_B_NED[0], savePlot, 2);
    pMap->addLogVar("q1 State Error", &pStateError->q_B_NED[1], savePlot, 2);
    pMap->addLogVar("q2 State Error", &pStateError->q_B_NED[2], savePlot, 2);
    pMap->addLogVar("q3 State Error", &pStateError->q_B_NED[3], savePlot, 2);
    pMap->addLogVar("Roll State Error (deg)", &pStateError->eulerAngles[0], savePlot, 2);
    pMap->addLogVar("Pitch State Error (deg)", &pStateError->eulerAngles[1], savePlot, 2);
    pMap->addLogVar("Yaw State Error (deg)", &pStateError->eulerAngles[2], savePlot, 2);
    pMap->addLogVar("Vel N State Error", &pStateError->velNED[0], savePlot, 2);
    pMap->addLogVar("Vel E State Error", &pStateError->velNED[1], savePlot, 2);
    pMap->addLogVar("Vel D State Error", &pStateError->velNED[2], savePlot, 2);
    pMap->addLogVar("Lat State Error (deg)", &pStateError->position[0], savePlot, 2);
    pMap->addLogVar("Lon State Error (deg)", &pStateError->position[1], savePlot, 2);
    pMap->addLogVar("Alt State Err (m)"    , &pStateError->position[2], savePlot, 2);
     */
#endif
    
    // Controls
    //pMap->addLogVar("da"          , &pControlData->da     , savePlot, 2);
    //pMap->addLogVar("de"          , &pControlData->de     , savePlot, 2);
    //pMap->addLogVar("dr"          , &pControlData->dr     , savePlot, 2);
    //pMap->addLogVar("Ctrl PMW [0]", &pControlData->TPWM[0], savePlot, 2);
    //pMap->addLogVar("Ctrl PMW [1]", &pControlData->TPWM[1], savePlot, 2);
    //pMap->addLogVar("Ctrl PMW [2]", &pControlData->TPWM[2], savePlot, 2);
    //pMap->addLogVar("Ctrl PMW [3]", &pControlData->TPWM[3], savePlot, 2);
}
#endif

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
    Wire.begin();
    display("Setup Begin.\n");
    
    // LED Pin
    pinMode(LEDPIN, OUTPUT);
    
    // IMU
    FsImu_setupIMU(accSensitivity, gyroSensitivity);
    
    // Barometer
    FsBarometer_setupBarometer();
    FsBarometer_setPressureResolution(bmp180Resolution);
    
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

void printData()
{
    #ifdef PRINT
    bool anyPrint = false;
    /*
    if (printTiming)
    {
        display(getTime());
        display(" ");
        
        display("50hz rate: ");
        display( 1.0/actualDelays[hz50] );
        display(", ");
        
        display("100hz rate: ");
        display( 1.0/actualDelays[hz100] );
        display(", ");
        
        display("200hz rate: ");
        display( 1.0/actualDelays[hz200] );
        display(", ");
        
        display("800hz rate: ");
        display( 1.0/actualDelays[hz800] );
        display("\n");
        
        anyPrint = true;
    }
    */
    if (printIMU)
    {
        display(getTime());
        display(" ");
        
        display("gyro raw (deg/s): ");
        display(pIMUdata->gyro[0]);
        display(" ");
        display(pIMUdata->gyro[1]);
        display(" ");
        display(pIMUdata->gyro[2]);
        display(", ");
        
        display("accel raw (g): ");
        display(pIMUdata->accel[0]);
        display(" ");
        display(pIMUdata->accel[1]);
        display(" ");
        display(pIMUdata->accel[2]);
        display("\n");
        
        anyPrint = true;
    }
    /*
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
        display(pNavData->accBias[0]/pNavData->gravity);
        display(" ");
        display(pNavData->accBias[1]/pNavData->gravity);
        display(" ");
        display(pNavData->accBias[2]/pNavData->gravity);
        display("\n");
        
        printCalibration = false;
        
        anyPrint = true;
    }
    
    if (printCalibratedIMU)
    {
        display(getTime());
        display(" ");
        
        display("gyro (deg/s): ");
        display(pIMUdata->gyro[0] - pNavData->gyroBias[0]*radian2degree);
        display(" ");
        display(pIMUdata->gyro[1] - pNavData->gyroBias[1]*radian2degree);
        display(" ");
        display(pIMUdata->gyro[2] - pNavData->gyroBias[2]*radian2degree);
        display(", ");
        
        display("accel (g): ");
        display(pIMUdata->accel[0] - pNavData->accBias[0]/pNavData->gravity);
        display(" ");
        display(pIMUdata->accel[1] - pNavData->accBias[1]/pNavData->gravity);
        display(" ");
        display(pIMUdata->accel[2] - pNavData->accBias[2]/pNavData->gravity);
        display("\n");
        
        anyPrint = true;
    }
    */
    if (printBarometer)
    {
        display(getTime());
        display(" ");
        
        display("Baro timestamp/pressure/temperature/altitude: ");
        display(pBaroData->timestamp);
        display(" ");
        display(pBaroData->pressure);
        display(" ");
        display(pBaroData->temperature);
        display(" ");
        display(pBaroData->altitude);
        display("\n");
    }
    /*
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
        
        display("velBody (m/s): ");
        display(pNavData->velBody[0]);
        display(" ");
        display(pNavData->velBody[1]);
        display(" ");
        display(pNavData->velBody[2]);
        display(", ");
        
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
        
        display("Lat/Lon/Alt (deg/deg/m): ");
        display(pNavData->position[0]*radian2degree);
        display(" ");
        display(pNavData->position[1]*radian2degree);
        display(" ");
        display(pNavData->position[2]*radian2degree);
        display("\n");
        
        anyPrint = true;
    }
    
    if (printPWMIn)
    {
        display(getTime());
        display(" ");
        
        display("pwmIn: ");
        display(pControlData->pwmCmd[THROTTLE]);
        display(" ");
        display(pControlData->pwmCmd[ROLL]);
        display(" ");
        display(pControlData->pwmCmd[PITCH]);
        display(" ");
        display(pControlData->pwmCmd[YAW]);
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
    */
    if (anyPrint) { display("\n"); }
    #endif
}
