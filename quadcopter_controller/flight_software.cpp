//
//  flight_software.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "flight_software.hpp"
#include <math.h>

#ifdef SIMULATION
    #include "imu_model.hpp"
    #include "time.hpp"
    #include "actuator_model.hpp"
    #include "dynamics_model.hpp"
    #include "model_mapping.hpp"
#else
    #include "Servo.h"
    #include "Wire.h" // This library allows you to communicate with I2C devices.
    #define EI_ARDUINO_INTERRUPTED_PIN
    #include <EnableInterrupt.h>
#endif

const double deg2rad = M_PI/180.0;

// Arduino Specific Variables
// I2C Registers
const int MPU_ADDR   = 0x68;
const int PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register
const int GYRO_REG   = 0x1B; // Gryo register
const int ACC_REG    = 0x1C; // Accelerometer register
const int ACC_OUT    = 0x3B; // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
const int INT        = 0x37;
const int INT_ENABLE = 0x38; //

// LED Pin
#ifndef SIMULATION
const unsigned int LEDPIN = LED_BUILTIN;
#endif

// Pulse In Pins
unsigned long TIMEOUT = 50000;
const unsigned int THROTTLEPIN = 7;  // CH3
const unsigned int ROLLPIN     = 8;  // CH4
const unsigned int PITCHPIN    = 12; // CH5
const unsigned int YAWPIN      = 13; // CH6

// Pulse Out Pins
const unsigned int T1PIN = 6;
const unsigned int T2PIN = 9;
const unsigned int T3PIN = 10;
const unsigned int T4PIN = 11;

const unsigned long PWMMIN = 1000;
const unsigned long PWMMAX = 2000;

// Interrupts
volatile unsigned int interruptCount = 0;
const unsigned int GPSintPIN = 3;

volatile bool GPSready = false;
volatile double gpsDT = 0.0;

// PwmIn Initiale Variables
#ifndef SIMULATION
int PwmIn::iPin  = 0;
int PwmIn::nPins = 0;
int PwmIn::pinArray[PwmIn::maxPins] = {0};
volatile int PwmIn::tRise = 0;
volatile int PwmIn::pwm[PwmIn::maxPins] = {0};
#endif

PwmIn throttleChannel;
PwmIn rollChannel;
PwmIn yawChannel;
PwmIn pitchChannel;

Servo T1esc;
Servo T2esc;
Servo T3esc;
Servo T4esc;

double T1PWMprint;
double T2PWMprint;
double T3PWMprint;
double T4PWMprint;

unsigned long T1PWM;
unsigned long T2PWM;
unsigned long T3PWM;
unsigned long T4PWM;

long baudRate;
bool controlAttitude = false;
bool intiailized     = false;
bool doneCalibrating = false;
bool printAngles     = false;
bool printIMU        = false;
bool printIMUcal     = false;
bool printPulseIn    = false;
bool printCalibration = false;
class IMUModelBase*  pIMU  = 0;
class ActuatorModelBase* pAct = 0;
class Time*          pTime = 0;
class DynamicsModel* pDyn  = 0;
class ModelMap*      pMap  = 0;

// IMU Sensitivity
enum accSensitivityType  {accSensitivity_2g, accSensitivity_4g, accSensitivity_8g, accSensitivity_16g, nAccSensitivity};
enum gyroSensitivityType {gyroSensitivity_250dps, gyroSensitivity_500dps, gyroSensitivity_1000dps, gyroSensitivity_2000dps, nGyroSensitivity};

double accSensitivityLSB[nAccSensitivity]  = {16483, 8192, 4096, 2048};
double gyroSensitivityLSB[nGyroSensitivity] = {131, 65.5, 32.8, 16.4};

char sensitivityByte[nAccSensitivity] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};

// Time keeping (s)
enum {hz50, hz100, printRoutine, nRoutines};
double prevTime[nRoutines];
double routineDelays[nRoutines];
bool   performRoutine[nRoutines];
double actualDelays[nRoutines];

// Event keeping (s)
enum {startGNC, imuWarmup, nEvents};
double eventStartTimes[nEvents];
bool eventStarted[nEvents];

// IMU
bool useIMU = false;
accSensitivityType  accSensitivity;
gyroSensitivityType gyroSensitivity;
char accSensitivityWrite;
char gyroSensitivityWrite;
bool newIMUmeasurement = false;

double accBias[3];
double gyroBias[3];
double magBias[3];

double accelerometerData[3];
double gyroscopeData[3];

double accCal[3];
double gyroCal[3];
double temperature;

double LSBdps;
double LSBg;
double LSBuT;

// Sensor Fusion (deg)
double attitude[3];
double eulerRates[3];
double eulerRatesDps[3];
double attitudeDeg[3];
double compFilterGain;

// Use accelemereometer when acceleration magnitude is below maxFilterAcc and above minFilterAcc
// and when body rate magnitude is below bodyRateThresh
double maxFilterAcc;
double minFilterAcc;
double bodyRateThresh;
bool useAcc;

double sr;
double cr;
double sp;
double tp;
double secp;
double aMag;
double bodyRateMag;

double attitudeFilterDt;

// Attitude Keeping
double rollCmdRaw;
double pitchCmdRaw;
double bodyZrateCmdRaw;

DiscreteCommand rollCmd;
DiscreteCommand pitchCmd;
DiscreteCommand bodyZrateCmd;

double rollCmdDeg;
double pitchCmdDeg;
double bodyZrateCmdDps;

double minDegree; // minimum change in command degree
double minDps;    // minimum change in command rate

double throttlePrint;
unsigned long throttle;
unsigned long droll;
unsigned long dpitch;
unsigned long dyaw;

const double MAXROLL     = 20 * deg2rad; // rad
const double MAXPITCH    = 20 * deg2rad; // rad
const double MAXYAWRATE  = 45 * deg2rad; // rad/s
const unsigned long MAXTHROTTLE = (PWMMAX-PWMMIN)*1.85; // PWM

// Errors
double attitudeError[3];
double attitdeCmdError[3];
double eulerRateError[3];
double bodyRateError[3];
double accelError[3];

double prev50hzTime = 0.0;
double cur50hzTime  = 0.0;
double dt50hz = 0.02;
int nSamples = 0.0;
double dtSum = 0.0;
double avgDt = 0.02;

double timeBefore = 0.0;
double timeAfter  = 0.0;
double neededDt = 0.0;

// **********************************************************************
// Initialize
// **********************************************************************
void initialize(void)
{
    // Setup
    initializeVariables();
    
    // Change These As Desired
    compFilterGain  = 0.01;
    accSensitivity  = accSensitivity_4g;
    gyroSensitivity = gyroSensitivity_500dps;
    baudRate        = 9600; // 9600, 38400, 115200
    maxFilterAcc    = 1.05;
    minFilterAcc    = 0.95;
    bodyRateThresh  = 1.0;
    minDegree       = 0.5;
    minDps          = 1.0;
    controlAttitude = false;
    useIMU          = true;
    printCalibration = false;
    printIMU        = false;
    printIMUcal     = false;
    printAngles     = false;
    printPulseIn    = false;
    eventStartTimes[imuWarmup] = 1.0;
    routineDelays[hz50] = 0.02;
    routineDelays[hz100] = 0.01;
    routineDelays[printRoutine] = 0.5;
    
    // Finish Setup
    getSimulationModels();
    setupArduino();
    
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

    // 50 hz routine
    if (performRoutine[hz50])
    {
        // Timing info
        actualDelays[hz50] = getTime() - prevTime[hz50];
        prevTime[hz50] = getTime();
        
        // Get IMU Data
        if (useIMU)
        {
            getImuData(accelerometerData, gyroscopeData, &temperature);
        }
        
        if ( eventStarted[imuWarmup] )
        {
            // Calibrate IMU Data
            doneCalibrating = groundCalibration(accBias, gyroBias, accelerometerData, gyroscopeData);
            
            if (doneCalibrating)
            {
                // Apply calibration
                for (int i=0; i<3; i++)
                {
                    accCal[i]  = accelerometerData[i] - accBias[i];
                    gyroCal[i] = gyroscopeData[i] - gyroBias[i];
                }
                
                // Roll, pitch, yaw
                attitudeFilterDt = routineDelays[hz50];
                atittudeFilter(attitude, accCal, gyroCal);
                
                // Control commands
                attitudeControl(attitude);
            }
        }
    }

    if (printIMU && performRoutine[printRoutine])
    {
        prevTime[printRoutine] = getTime();

        display("Acc Gyro: ");
        display( *(accelerometerData+0) );
        display(" ");
        display( *(accelerometerData+1) );
        display(" ");
        display( *(accelerometerData+2) );
        
        display(" ");
        display( *(gyroscopeData+0) );
        display(" ");
        display( *(gyroscopeData+1) );
        display(" ");
        display( *(gyroscopeData+2) );
        display("\n");
    }

    computeErrors();
    
    return true;
}

// **********************************************************************
// IMU
// **********************************************************************
void getImuData(double* acc, double* gyro, double* temperature)
{
    // Get Raw Values
#ifdef SIMULATION
    if (pIMU)
    {
        for (int i=0; i<3; i++)
        {
            acc[i]  = pIMU->getAccelerometer()[i];
            gyro[i] = pIMU->getGyroscope()[i];
        }
    }
#else
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACC_OUT); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers
    
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    if ( Wire.available() )
    {
    *(acc+0)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    *(acc+1)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    *(acc+2)       = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    *(temperature) = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    *(gyro+0)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    *(gyro+1)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    *(gyro+2)      = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    }
    else if (performRoutine[printRoutine])
    {
        display("No I2C\n");
    }
#endif
    
    // Convert to Units
    for (int i=0; i<3; i++)
    {
        *(acc+i)  /= LSBg;
        *(gyro+i) /= LSBdps;
    }
}

bool groundCalibration(double* accBias, double* gyroBias, double* acc, double* gyro)
{
    static bool doneCalibrating = false;
    static int iSample   = 0;
    const int  maxSample = 100;
    
    static double accSum[3];
    static double gyroSum[3];
    
    if ( !doneCalibrating )
    {
        if (printCalibration && performRoutine[printRoutine])
        {
            display("Calibarting IMU\n");
            
            display("(");
            display(getTime());
            display(" ");
            display(iSample);
            display(") ");
            
            display("acc gyro: ");
            display( *(acc+0) );
            display(" ");
            display( *(acc+1) );
            display(" ");
            display( *(acc+2) );
            display(" ");

            display( *(gyro+0) );
            display(" ");
            display( *(gyro+1) );
            display(" ");
            display( *(gyro+2) );
            display("\n");
        }
        
        accSum[0]  += acc[0];
        accSum[1]  += acc[1];
        accSum[2]  += acc[2] - 1.0;
        gyroSum[0] += gyro[0];
        gyroSum[1] += gyro[1];
        gyroSum[2] += gyro[2];
        iSample++;

        if (iSample >= maxSample)
        {
            for (int i=0; i<3; i++)
            {
                *(accBias+i)  = accSum[i]/maxSample;
                *(gyroBias+i) = gyroSum[i]/maxSample;
            }
            
            display("Done calibrating IMU\n");
            
            display("(");
            display(getTime());
            display(") ");
            display("accBias: ");
            display( *(accBias+0) );
            display("\t");
            display( *(accBias+1) );
            display("\t");
            display( *(accBias+2) );
            display("\n");
            
            display("(");
            display(getTime());
            display(") ");
            display("gyroBias: ");
            display( *(gyroBias+0) );
            display("\t");
            display( *(gyroBias+1) );
            display("\t");
            display( *(gyroBias+2) );
            display("\n");
            
#ifndef SIMULATION
            LEDon();
#endif
            doneCalibrating = true;
        }
    }
    
    return doneCalibrating;
}

// **********************************************************************
// GPS
// **********************************************************************
void GPSInterrupt()
{
    // TODO
}

// **********************************************************************
// Algorithms
// **********************************************************************
void atittudeFilter(double* attitude, double* acc, double* gyro)
{
    // Variables
    double eulerPredGyro[3];
    double eulerPredAcc[3];
    
    sr   = sin(attitude[0]);
    cr   = cos(attitude[0]);
    sp   = cos(attitude[1]);
    tp   = tan(attitude[1]);
    secp = 1.0/cos(attitude[1]);
    
    // Euler rates from gyroscope
    /*  -------------------------------------------------------------------------------
     * Body rates to euler rates matrix transformation
     *   euler_rates   =         L               x   body_rates
     *  [ roll_rate  ]   [ 1   sr*tp    cr*sp  ]    [ body_rate_x ]
     *  [ pitch_rate ] = [ 0    cr       -sr   ] x  [ body_rate_y ]
     *  [ yaw_rate   ]   [ 0  sr*secp  cr*secp ]    [ body_rate_z ]
     * -------------------------------------------------------------------------------
     */
    eulerRates[0] = ( gyro[0] - sr*tp*gyro[1]   -  cr*sp*gyro[2]   ) * deg2rad;
    eulerRates[1] = (         - cr*gyro[1]      +  sr*gyro[2]      ) * deg2rad;
    eulerRates[2] = (         - sr*secp*gyro[1] -  cr*secp*gyro[2] ) * deg2rad;
        
    // Use gyro to compute angles
    eulerPredGyro[0] = attitude[0] + eulerRates[0] * attitudeFilterDt;
    eulerPredGyro[1] = attitude[1] + eulerRates[1] * attitudeFilterDt;
    eulerPredGyro[2] = attitude[2] + eulerRates[2] * attitudeFilterDt;
    
    // Use accelerometer to measure angles
    eulerPredAcc[0] = atan2( acc[1], acc[2] );
    eulerPredAcc[1] = atan2( acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]) );
    
    // Body Rate magnitude
    bodyRateMag = sqrt( gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2] );
    
    // Acceleration magnitude
    aMag = sqrt( acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2] );
    
    useAcc = (aMag < maxFilterAcc) && (aMag > minFilterAcc) && (bodyRateMag < bodyRateThresh);
    
    // Estimate angles
    if (useAcc)
    {
        attitude[0]  = eulerPredGyro[0]  + compFilterGain * (eulerPredAcc[0] - eulerPredGyro[0]);
        attitude[1]  = eulerPredGyro[1]  + compFilterGain * (eulerPredAcc[1] - eulerPredGyro[1]);
    }
    else
    {
        attitude[0] = eulerPredGyro[0];
        attitude[1] = eulerPredGyro[1];
    }
    attitude[2]  = eulerPredGyro[2];
    
    for (int i=0; i<3; i++)
    {
        attitudeDeg[i] = attitude[i] / deg2rad;
        eulerRatesDps[i] = eulerRates[i] / deg2rad;
    }
    
    if (printAngles && performRoutine[printRoutine])
    {
        display("(");
        display(getTime());
        display(") ");
        display("Roll Pitch Yaw: ");
        display( attitude[0]/deg2rad );
        display(" ");
        display( attitude[1]/deg2rad );
        display(" ");
        display( attitude[2]/deg2rad );
        display(" ");
        display( useAcc );
        display("\n");
    }
}

void attitudeControl(double* attitude)
{
    // Get Desired Aittutde
    unsigned long throttlePWM = throttleChannel.getPwm();
    unsigned long rollPWM     = rollChannel.getPwm();
    unsigned long pitchPWM    = pitchChannel.getPwm();
    unsigned long yawPWM      = yawChannel.getPwm();
    
    throttle = mapToValue(throttlePWM, PWMMIN, PWMMAX, PWMMIN, MAXTHROTTLE);
    rollCmdRaw = mapToValue(rollPWM, PWMMIN, PWMMAX, -MAXROLL, MAXROLL);      // Roll (rad)
    pitchCmdRaw = mapToValue(pitchPWM, PWMMIN, PWMMAX, -MAXPITCH, MAXPITCH);   // Pitch (rad)
    bodyZrateCmdRaw = mapToValue(yawPWM, PWMMIN, PWMMAX, -MAXYAWRATE, MAXYAWRATE); // Yaw Rate (rad/s)
  
    // Discretize Commands
    rollCmd.setCommand( rollCmdRaw );
    pitchCmd.setCommand( pitchCmdRaw );
    bodyZrateCmd.setCommand( bodyZrateCmdRaw );
    
    // Controller
    if (controlAttitude)
    {
        
    }
    else
    {
        droll  = 0;
        dpitch = 0;
        dyaw   = 0;
    }
    
    T1PWM = limit(throttle - droll + dpitch + dyaw, PWMMIN, PWMMAX);
    T2PWM = limit(throttle + droll + dpitch - dyaw, PWMMIN, PWMMAX);
    T3PWM = limit(throttle + droll - dpitch + dyaw, PWMMIN, PWMMAX);
    T4PWM = limit(throttle - droll - dpitch - dyaw, PWMMIN, PWMMAX);
    
    T1esc.writeMicroseconds(T1PWM);
    T2esc.writeMicroseconds(T2PWM);
    T3esc.writeMicroseconds(T3PWM);
    T4esc.writeMicroseconds(T4PWM);
    
    // House Keeping
    throttlePrint = static_cast<double> (throttle);
    T1PWMprint = static_cast<double> (T1PWM);
    T2PWMprint = static_cast<double> (T2PWM);
    T3PWMprint = static_cast<double> (T3PWM);
    T4PWMprint = static_cast<double> (T4PWM);
    
    rollCmdDeg = rollCmd.value() / deg2rad;
    pitchCmdDeg = pitchCmd.value() / deg2rad;
    bodyZrateCmdDps = bodyZrateCmd.value() / deg2rad;
    
    if (printPulseIn && performRoutine[printRoutine])
    {
        display("Commands: ");
        display(throttle);
        display(" ");
        display(rollCmdDeg);
        display(" ");
        display(pitchCmdDeg);
        display(" ");
        display(bodyZrateCmdDps);
        display("\n");
    }

}

DiscreteCommand::DiscreteCommand()
{
    command = 0.0;
    lengthArray = 0;
    increasing = true;
    
    minCmd = 0.0;
    maxCmd = 0.0;
    minIncr = 0.0;
}

void DiscreteCommand::setup(double minIncrIn, double minCmdIn, double maxCmdIn)
{
    minCmd = minCmdIn;
    maxCmd = maxCmdIn;
    minIncr = minIncrIn;
    lengthArray = (maxCmd - minCmd) / minIncr;
}

void DiscreteCommand::setCommand(double desiredCommand)
{
    if (minCmd == maxCmd)
    {
        display("Warning: command not setup!\n");
        return;
    }
    
    // Determine increasing/decreasing
    if (desiredCommand >= command) { increasing = true; }
    else { increasing = false; }
    
    if (increasing)
    {
        for (int i=0; i<lengthArray; i++)
        {
            updateCommandOption(i);
            if ( desiredCommand >= commandOption )
            {
                command = commandOption;
            }
        }
    }
    else
    {
        for (int i=lengthArray-1; i>-1; i--)
        {
            updateCommandOption(i);
            if ( desiredCommand <= commandOption )
            {
                command = commandOption;
            }
        }
    }
}

void DiscreteCommand::updateCommandOption(int index)
{
    commandOption = minCmd + minIncr*index;
}

// **********************************************************************
// Initialization Functions
// **********************************************************************
void getSimulationModels(void)
{
# ifdef SIMULATION
    if(pMap)
    {
        pIMU  = (IMUModelBase*)      pMap->getModel("IMUModel");
        pAct  = (ActuatorModelBase*) pMap->getModel("ActuatorModel");
        pTime = (Time*)              pMap->getModel("Time");
        pDyn  = (DynamicsModel*)     pMap->getModel("DynamicsModel");
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
    
    pMap->addLogVar("FS Acc X" , &accelerometerData[0], savePlot, 2);
    pMap->addLogVar("FS Acc Y" , &accelerometerData[1], savePlot, 2);
    pMap->addLogVar("FS Acc Z" , &accelerometerData[2], savePlot, 2);
    pMap->addLogVar("FS Acc Mag" , &aMag, printSavePlot, 3);
    pMap->addLogVar("FS Gyro X", &gyroscopeData[0], savePlot, 2);
    pMap->addLogVar("FS Gyro Y", &gyroscopeData[1], savePlot, 2);
    pMap->addLogVar("FS Gyro Z", &gyroscopeData[2], savePlot, 2);
    
    //pMap->addLogVar("sr", &sr, savePlot, 2);
    //pMap->addLogVar("cr", &cr, savePlot, 2);
    //pMap->addLogVar("sp", &sp, savePlot, 2);
    //pMap->addLogVar("tp", &tp, savePlot, 2);
    //pMap->addLogVar("secp", &secp, savePlot, 2);
    
    pMap->addLogVar("FS Roll Rate", &eulerRatesDps[0], savePlot, 2);
    pMap->addLogVar("FS Pitch Rate", &eulerRatesDps[1], savePlot, 2);
    pMap->addLogVar("FS Yaw Rate", &eulerRatesDps[2], savePlot, 2);
    pMap->addLogVar("attitudeFilterDt", &attitudeFilterDt, savePlot, 2);
    pMap->addLogVar("FS Roll Rate Error"  , &eulerRateError[0], savePlot, 2);
    pMap->addLogVar("FS Pitch Rate Error"  , &eulerRateError[1], savePlot, 2);
    pMap->addLogVar("FS Yaw Rate Error"  , &eulerRateError[2], savePlot, 2);
    
    //pMap->addLogVar("FS Acc X Bias" , &accBias[0], savePlot, 2);
    //pMap->addLogVar("FS Acc Y Bias" , &accBias[1], savePlot, 2);
    //pMap->addLogVar("FS Acc Z Bias" , &accBias[2], savePlot, 2);
    //pMap->addLogVar("FS Gyro X Bias", &gyroBias[0], savePlot, 2);
    //pMap->addLogVar("FS Gyro Y Bias", &gyroBias[1], savePlot, 2);
    //pMap->addLogVar("FS Gyro Z Bias", &gyroBias[2], savePlot, 2);
    pMap->addLogVar("FS Throttle Cmd", &throttlePrint, savePlot, 2);
    pMap->addLogVar("FS Roll Cmd" , &rollCmdDeg, savePlot, 2);
    pMap->addLogVar("FS Pitch Cmd", &pitchCmdDeg, savePlot, 2);
    pMap->addLogVar("FS Yaw Cmd"  , &bodyZrateCmdDps, savePlot, 2);
    
    pMap->addLogVar("FS Roll" , &attitudeDeg[0], printSavePlot, 3);
    pMap->addLogVar("FS Pitch", &attitudeDeg[1], printSavePlot, 3);
    pMap->addLogVar("FS Yaw"  , &attitudeDeg[2], printSavePlot, 3);
    
    pMap->addLogVar("FS Roll Error"  , &attitudeError[0], savePlot, 2);
    pMap->addLogVar("FS Pitch Error"  , &attitudeError[1], savePlot, 2);
    pMap->addLogVar("FS Yaw Error"  , &attitudeError[2], savePlot, 2);
    
    pMap->addLogVar("FS T1PWM"  , &T1PWMprint, savePlot, 2);
    pMap->addLogVar("FS T2PWM"  , &T2PWMprint, savePlot, 2);
    pMap->addLogVar("FS T3PWM"  , &T3PWMprint, savePlot, 2);
    pMap->addLogVar("FS T4PWM"  , &T4PWMprint, savePlot, 2);
    
#endif
}

void computeErrors()
{
#ifdef SIMULATION
    for (int i=0; i<3; i++)
    {
        if (pDyn)
        {
            attitudeError[i] = ( attitude[i] - pDyn->getEulerAngles()[i].rad() ) / deg2rad;
            eulerRateError[i] = ( eulerRates[i] - pDyn->getEulerRates()[i].rps() ) / deg2rad;
        }
    }
#endif
}

double getTime()
{
#ifdef SIMULATION
    if (pTime) { return pTime->getSimTime(); }
    else { return 0.0; }
#else
    return micros() / 1000000.0;
#endif
}

template<typename TempType>
void display(TempType val)
{
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
}

template void display(String);
template void display(int);
template void display(float);
template void display(double);

// **********************************************************************
// PwmIn Class
// **********************************************************************
#ifdef SIMULATION
PwmIn::PwmIn()
{
    // Initialize Pins
    Pins[THROTTLE] = THROTTLEPIN; // CH3
    Pins[ROLL]     = ROLLPIN;     // CH4
    Pins[PITCH]    = PITCHPIN;    // CH5
    Pins[YAWRATE]  = YAWPIN;      // CH6
    
    // Initialize Ranges
    minValues[THROTTLE] = 0.0;
    minValues[ROLL]     = -MAXROLL;
    minValues[PITCH]    = -MAXPITCH;
    minValues[YAWRATE]  = -MAXYAWRATE;
    
    maxValues[THROTTLE] = 100.0;
    maxValues[ROLL]     = MAXROLL;
    maxValues[PITCH]    = MAXPITCH;
    maxValues[YAWRATE]  = MAXYAWRATE;
    
    // Initialize Variables
    channel  = THROTTLE;
    pwm      = PWMMIN;
    minValue = minValues[channel];
    maxValue = maxValues[channel];
    value    = minValue;
    pTable   = NULL;
    foundChannel = false;
    
    // Table To Radians
    for (int iCh = ROLL; iCh != nChannels; iCh++)
    {
        for (int col = 0; col < lengthTable; col++)
        {
            signalValues[iCh][col] *= deg2rad;
        }
    }
}

void PwmIn::attach(int pinIn)
{
    // Find Channel
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        if (!foundChannel && pinIn == Pins[iCh])
        {
            channel = static_cast<channelType> (iCh);
            foundChannel = true;
        }
    }
    
    if (foundChannel)
    {
        minValue = minValues[channel];
        maxValue = maxValues[channel];
        value = minValue;
        
        pTable = new tableType;
        pTable->pTableTimes  = &signalTimes[channel][0];
        pTable->pTableValues = &signalValues[channel][0];
        pTable->tableLength  = lengthTable;
    }
    else
    {
        display("PwmIn::attach, channel not found.\n");
    }
}

unsigned long PwmIn::getPwm()
{
    if (pTable == NULL) return pwm;
    
    for (int i=0; i<pTable->tableLength; i++)
    {
        /*
        display("Time: ");
        display( getTime() );
        display(" Table Time: ");
        display( pTable->pTableTimes[i] );
        display(" Table Value: ");
        display( pTable->pTableValues[i] );
        display("\n");
        */
        if( getTime() >= pTable->pTableTimes[i] )
        {
            value = pTable->pTableValues[i];
        }
    }
    /*
    display(" Value: ");
    display( value );
    display("\n");
    */
    pwm = mapToPwm(value, minValue, maxValue, PWMMIN, PWMMAX);
    
    return pwm;
}

#else
PwmIn::PwmIn()
{
    thisPin = -1;
    thisPinLoc = -1;
}

void PwmIn::attach(int pinIn)
{
    // Store Pin
    thisPin = pinIn;
    thisPinLoc = nPins;
    
    // Initialize Arrays
    pwm[thisPinLoc] = 1000;
    pinArray[thisPinLoc] = thisPin;
    
    // Setup Pin
    pinMode(thisPin, INPUT_PULLUP);
    
    // Initialize Variables
    if (nPins == 0)
    {
        iPin = 0;
        tRise = 0;
        enableInterrupt(pinArray[0], PwmIn::riseInterrupt, RISING);
    }
    
    // Increment Pins
    nPins++;
}
unsigned long PwmIn::getPwm()
{
    if (thisPinLoc != -1) { return pwm[thisPinLoc]; }
    
    // Calling getPwm before attached
    else { return 0; }
}

void PwmIn::riseInterrupt()
{
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        tRise = micros();
        enableInterrupt(pinArray[iPin], PwmIn::fallInterrupt, FALLING);
    }
}

void PwmIn::fallInterrupt()
{
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        int tempPWM = micros() - tRise;
        if (tempPWM >= minPWM && tempPWM <= maxPWM)
        {
            pwm[iPin] = tempPWM;
        }
        disableInterrupt(pinArray[iPin]);
        
        iPin++;
        if (iPin >= nPins) { iPin = 0; }
        enableInterrupt(pinArray[iPin], PwmIn::riseInterrupt, RISING);
    }
}

void LEDon()  { digitalWrite(LEDPIN, HIGH); }
void LEDoff() { digitalWrite(LEDPIN, LOW); }

#endif

#ifdef SIMULATION
Servo::Servo()
{
    motorPins[T1] = T1PIN;
    motorPins[T2] = T2PIN;
    motorPins[T3] = T3PIN;
    motorPins[T4] = T4PIN;
    
    throttle = 0.0;
    foundMotor = false;
}

void Servo::attach(int pinIn)
{
    // Find Channel
    for (int iMotor = T1; iMotor != nMotors; iMotor++)
    {
        if (!foundMotor && pinIn == motorPins[iMotor])
        {
            motorNumber = static_cast<motorNumberType> (iMotor);
            foundMotor = true;
        }
    }
    
    if (!foundMotor)
    {
        display("Servo::attach, motor not found.\n");
    }
}

void Servo::writeMicroseconds(unsigned long pwm)
{
    throttle = mapToValue(pwm, PWMMIN, PWMMAX, 0.0, 1.0);
    if (pAct && foundMotor)
    {
        pAct->setCommands(throttle, motorNumber);
    }
}

double Servo::read()
{
    return throttle;
}

#endif

double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = (valueMax-valueMin) / static_cast<double>(pwmMax-pwmMin);
    return slope*(pwm-pwmMin) + valueMin;
}

unsigned long mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, unsigned long valueMin, unsigned long valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = 1.0*(valueMax-valueMin) / (pwmMax-pwmMin);
    return slope*(pwm-pwmMin) + valueMin;
}

unsigned long mapToPwm(double value, double valueMin, double valueMax, unsigned long pwmMin, unsigned long pwmMax)
{
    // y - y1 = m * (x - x1)
    double slope = (pwmMax-pwmMin) / (valueMax-valueMin);
    return slope*(value-valueMin) + pwmMin;
}

unsigned long limit(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax)
{
    if (pwm < pwmMin)
    {
        display("Warning: pwm below limit ");
        display(pwm);
        display("\n");
        return pwmMin;
    }
    if (pwm > pwmMax)
    {
        display("Warning: pwm above limit ");
        display(pwm);
        display("\n");
        return pwmMax;
    }
    return pwm;
}

// **********************************************************************
// Initialization
// **********************************************************************
void initializeVariables(void)
{
    useAcc = false;
    
    throttle = 0;
    droll    = 0;
    dpitch   = 0;
    dyaw     = 0;
    
    for (int i=0; i<3; i++)
    {
        accBias[i]  = 0.0;
        gyroBias[i] = 0.0;
        magBias[i]  = 0.0;
        accelerometerData[i] = 0.0;
        gyroscopeData[i]     = 0.0;
        accCal[i]  = 0.0;
        gyroCal[i] = 0.0;
        attitude[i]      = 0.0;
        eulerRates[i]    = 0.0;
        eulerRatesDps[i] = 0.0;
        attitudeDeg[i]   = 0.0;
    }
    
    LSBdps = 0.0;
    LSBg   = 0.0;
    LSBuT  = 0.0;
    temperature = 0.0;
    compFilterGain = 0.0;
    
    for (int i=0; i<nRoutines; i++)
    {
        prevTime[i] = 0.0;
        performRoutine[i] = false;
        routineDelays[i] = 0.0;
        actualDelays[i]  = 0.0;
    }
    
    for (int i=0; i<nEvents; i++)
    {
        eventStartTimes[i] = 0.0;
        eventStarted[i] = false;
    }
}

void setupArduino(void)
{
    LSBdps = gyroSensitivityLSB[gyroSensitivity];
    gyroSensitivityWrite = sensitivityByte[gyroSensitivity];
    
    LSBg = accSensitivityLSB[accSensitivity];
    accSensitivityWrite = sensitivityByte[accSensitivity];
    
    // PWM In
    throttleChannel.attach(THROTTLEPIN);
    rollChannel.attach(ROLLPIN);
    pitchChannel.attach(PITCHPIN);
    yawChannel.attach(YAWPIN);
    
    // Servos
    T1esc.attach(T1PIN);
    T2esc.attach(T2PIN);
    T3esc.attach(T3PIN);
    T4esc.attach(T4PIN);
    
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    
    // Commands
    rollCmd.setup(minDegree*deg2rad, -MAXROLL, MAXROLL);
    pitchCmd.setup(minDegree*deg2rad, -MAXPITCH, MAXPITCH);
    bodyZrateCmd.setup(minDps*deg2rad, -MAXYAWRATE, MAXYAWRATE);
    
#ifdef SIMULATION
    if (pIMU)
    {
        pIMU->setLSBdps(LSBdps);
        pIMU->setLSBg(LSBg);
    }
#else
    // Serial
    Serial.begin(baudRate);
    display("Setup Begin.\n");
    
    // LED Pin
    pinMode(LEDPIN, OUTPUT);
    
    // IMU I2C
    if (useIMU)
    {
        display("Setting Up I2C...\n");
        Wire.begin();
        Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
        Wire.write(PWR_MGMT_1); // PWR_MGMT_1 register
        Wire.write(0); // set to zero (wakes up the MPU-6050)
        Wire.endTransmission(true);
        
        // Gyro range
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_REG); // Gryo register
        Wire.write(gyroSensitivityWrite);
        Wire.endTransmission(true);
        
        // Accelerometer range
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACC_REG); // Accelerometer register
        Wire.write(accSensitivityWrite);
        Wire.endTransmission(true);
    }
    
    LEDoff();
    display("Setup Complete.\n");
#endif
}
