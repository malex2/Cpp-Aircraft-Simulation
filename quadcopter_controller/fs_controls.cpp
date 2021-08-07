//
//  fs_controls.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_controls.hpp"
#include "fs_imu.hpp"
#include "fs_navigation.hpp"
#include "fs_pwmin.hpp"

#ifndef SIMULATION
    #include "Servo.h"
#endif

ControlType controlData;

// Servos
Servo T1esc;
Servo T2esc;
Servo T3esc;
Servo T4esc;

int pwmCmd[nChannels];
int prevPwmCmd[nChannels];

// Navigation Data
double eulerAngles[3];
double velLL[3];

// IMU Data
double bodyRates[3];

// Temp Limits for Monitoring
double zeroLimitMin;
double zeroLimitMax;
double maxLimitMin;
double maxLimitMax;

// Limits
int minPWM;
int maxPWM;
double minThrottle;
double maxThrottle;
double minRPM;
double maxRPM;

// Gains
// Velocity Channels
#define kp_vx  1.0
#define kp_vy  1.0
#define kp_vz -1.0

// Attitude Channels
#define kp_roll  148473976
#define kd_roll  59105900
#define kp_pitch 148473976
#define kd_pitch 59105900
#define kp_yaw   248680042

bool controls_setup = false;

void FsControls_setup()
{
    // Servos
    T1esc.attach(T1PIN);
    T2esc.attach(T2PIN);
    T3esc.attach(T3PIN);
    T4esc.attach(T4PIN);
    
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    
    for (int i=0; i<3; i++)
    {
        bodyRates[i] = 0.0;
        velLL[i] = 0.0;
    }
    
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        pwmCmd[iCh]     = PWMMIN;
        prevPwmCmd[iCh] = PWMMIN;
    }
    controls_setup = true;
}

void FsControls_performControls()
{
    // Not enough initialization to start controls
    if ( (controlData.mode == NoControl) ||
        (!controls_setup) ||
        (FsNavigation_getNavState() == Calibration)
        )
    {
        return;
    }
    
    performControls();
    
    setMotors();
    
    controlData.timestamp = getTime();
}

void performControls()
{
    // Discritize Commands to Limit Unintentional Change
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        prevPwmCmd[iCh] = pwmCmd[iCh];
        pwmCmd[iCh] = discretize(pwmCmd[iCh], prevPwmCmd[iCh], PWMMIN, PWMMAX, minPWMΙncr);
    }
    
    // Compute Throttle PWM Limits
    rpmPwmLimits( (float) pwmCmd[THROTTLE], (float) PWMMINRPM );
    
    // Convert PWM to values
    if (controlData.mode == ThrottleControl)
    {
        controlData.dT = mapToValue(pwmCmd[THROTTLE], minPWM, maxPWM, minThrottle, maxThrottle);
    }
    
    if (controlData.mode == AttitudeControl)
    {
        controlData.dT         = mapToValue(pwmCmd[THROTTLE] ,       minPWM,       maxPWM,          minThrottle,      maxThrottle);
        controlData.rollCmd    = mapToValue(pwmCmd[ROLL]     , (int) PWMMIN, (int) PWMMAX, (double) -MAXROLL   , (double) MAXROLL);    // Roll (rad)
        controlData.pitchCmd   = mapToValue(pwmCmd[PITCH]    , (int) PWMMIN, (int) PWMMAX, (double) -MAXPITCH  , (double) MAXPITCH);   // Pitch (rad)
        controlData.yawRateCmd = mapToValue(pwmCmd[YAW]      , (int) PWMMIN, (int) PWMMAX, (double) -MAXYAWRATE, (double) MAXYAWRATE); // Yaw Rate (rad/s)
    }
    
    else if (controlData.mode == VelocityControl)
    {
        controlData.VLLzCmd    = mapToValue(pwmCmd[THROTTLE], (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.VLLyCmd    = mapToValue(pwmCmd[ROLL]    , (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.VLLxCmd    = mapToValue(pwmCmd[PITCH]   , (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.yawRateCmd = mapToValue(pwmCmd[YAW]     , (int) PWMMIN, (int) PWMMAX, (double) -MAXYAWRATE , (double) MAXYAWRATE);  // (rad/s)
        
        // Outer Velocity Loop to Generate Attitude Commands
        controlData.dT       = kp_vz*(controlData.VLLzCmd - velLL[2]) + controlData.dTo;
        controlData.rollCmd  = kp_vy*(controlData.VLLyCmd - velLL[1]);
        controlData.pitchCmd = kp_vx*(controlData.VLLxCmd - velLL[0]);
        
        // Limit Commands
        controlData.dT       = limit(controlData.dT, minThrottle, maxThrottle);
        controlData.rollCmd  = limit(controlData.rollCmd, (double) -MAXROLL, (double) MAXROLL);
        controlData.pitchCmd = limit(controlData.pitchCmd, (double) -MAXPITCH, (double) MAXPITCH);
    }
    
    // Control Attitude
    if (controlData.mode != ThrottleControl)
    {
        controlData.da = kp_roll *(controlData.rollCmd    - eulerAngles[0]) - kd_roll*bodyRates[0];
        controlData.de = kp_pitch*(controlData.pitchCmd   - eulerAngles[1]) - kd_pitch*bodyRates[1];
        controlData.dr = kp_yaw  *(controlData.yawRateCmd - bodyRates[2]);
        
        //std::cout << "dr: " << controlData.dr;
        //std::cout << " yawRateCmd: " << controlData.yawRateCmd/ double(degree2radian);
        //std::cout << " yawRate: " << bodyRates[2]/ double(degree2radian);
        //std::cout << " yawRateError: " << (controlData.yawRateCmd - bodyRates[2])/ double(degree2radian) << std::endl;
        
        // Increase throttle to allow attitude following
        minAttitudeThrottle();
        if (controlData.dT < minThrottle)
        {
            //display("Increasing throttle to ");
            //display(minThrottle);
            //display(" for attitude control.\n");
        }
        controlData.dT = limit(controlData.dT, minThrottle, maxThrottle);
    }
    else
    {
        controlData.da = 0.0;
        controlData.de = 0.0;
        controlData.dr = 0.0;
    }
    
    // Limit Commands
    // Prevent RPM's less than 0
    controlData.da = limit(controlData.da, -controlData.dT/3.0, controlData.dT/3.0);
    controlData.de = limit(controlData.de, -controlData.dT/3.0, controlData.dT/3.0);
    controlData.dr = limit(controlData.dr, -controlData.dT/3.0, controlData.dT/3.0);
    
    // Prevent RPM's greater than RPMMAX
    controlData.da = limit(controlData.da, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    controlData.de = limit(controlData.de, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    controlData.dr = limit(controlData.dr, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    
    zeroLimitMin = -controlData.dT/3.0;
    zeroLimitMax = controlData.dT/2.0;
    maxLimitMin  = (controlData.dT-dMAX)/2.0;
    maxLimitMax  = (dMAX-controlData.dT)/3.0;
}

void setMotors()
{
    double rpm;
    
    // Determine RPM's
    controlData.rpmSq[0] = (controlData.dT + controlData.da + controlData.de + controlData.dr)/4.0;
    controlData.rpmSq[1] = (controlData.dT - controlData.da + controlData.de - controlData.dr)/4.0;
    controlData.rpmSq[2] = (controlData.dT - controlData.da - controlData.de + controlData.dr)/4.0;
    controlData.rpmSq[3] = (controlData.dT + controlData.da - controlData.de - controlData.dr)/4.0;
    
    // Set Throttle PWM's
    for (int i=0; i<4; i++)
    {
        if (controlData.rpmSq[i] < 0.0)
        {
            controlData.rpmSq[i] = 0.0;
            display("WARNING: Prop ");
            display(i+1);
            display(" attempting RPM < 0!\n");
        }
        rpm = sqrt(controlData.rpmSq[i]);
        rpmPwmLimits( float(rpm), float(MINRPM) );
        controlData.TPWM[i] = mapToValue(rpm, minRPM, maxRPM, minPWM, maxPWM);
    }
    
    // Write to motors
    T1esc.writeMicroseconds( static_cast<int> (controlData.TPWM[0]) );
    T2esc.writeMicroseconds( static_cast<int> (controlData.TPWM[1]) );
    T3esc.writeMicroseconds( static_cast<int> (controlData.TPWM[2]) );
    T4esc.writeMicroseconds( static_cast<int> (controlData.TPWM[3]) );
}

void FsControls_setMode(ControlMode mode) { controlData.mode = mode; }

void FsControls_setPWMCommands(int* pwmIn)
{
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        pwmCmd[iCh] = pwmIn[iCh];
    }
}

void FsControls_setSimulationModels(ModelMap* pMap)
{
#ifdef SIMULATION
    T1esc.servo_setSimulationModels(pMap);
    T2esc.servo_setSimulationModels(pMap);
    T3esc.servo_setSimulationModels(pMap);
    T4esc.servo_setSimulationModels(pMap);
#endif
}

void FsControls_setControlsData(IMUtype* pIMUdataIn, NavType* pNavDataIn)
{
    for (int i=0; i<3; i++)
    {
        bodyRates[i]   = pIMUdataIn->gyro[i]*degree2radian - pNavDataIn->gyroBias[i];
        eulerAngles[i] = pNavDataIn->eulerAngles[i];
    }
    FsNavigation_bodyToLL(velLL, pNavDataIn->velBody);
}

ControlType* FsControls_getControlData()
{
    return &controlData;
}

int discretize(int desiredCommand, int command, int minCmd, int maxCmd, int deltaPwm)
{
    bool increasing = false;
    int lengthArray = (maxCmd - minCmd) / deltaPwm + 1;
    int tempCmd = minCmd;
    
    // Determine increasing/decreasing
    if (desiredCommand >= command) { increasing = true; }
    
    if (increasing)
    {
        for (int i=0; i<lengthArray; i++)
        {
            tempCmd = minCmd + deltaPwm*i;
            if ( desiredCommand >= tempCmd )
            {
                command = tempCmd;
            }
        }
    }
    else
    {
        for (int i=lengthArray-1; i>-1; i--)
        {
            tempCmd = minCmd + deltaPwm*i;
            if ( desiredCommand <= tempCmd )
            {
                command = tempCmd;
            }
        }
    }
    return command;
}

void rpmPwmLimits(float value, float threshold)
{
    if (value < threshold)
    {
        minPWM = PWMMIN;
        maxPWM = PWMMINRPM;
        minRPM = 0.0;
        maxRPM = MINRPM;
        minThrottle = 0.0;
        maxThrottle = MINTHROTTLE;
    }
    else
    {
        minPWM = PWMMINRPM;
        maxPWM = PWMMAX;
        minRPM = MINRPM;
        maxRPM = MAXRPM;
        minThrottle = MINTHROTTLE;
        maxThrottle = MAXTHROTTLE;
    }
}

void minAttitudeThrottle()
{
    double dmax;
    double de;
    double dr;
    
    // Get maximum of attitude commands
    dmax = fabs( controlData.da );
    de = fabs( controlData.de );
    dr = fabs( controlData.dr );
    
    if (de > dmax) { dmax = de; }
    if (dr > dmax) { dmax = dr; }
    
    // Minimum throttle to satisfy all commands without 0 RPM
    minThrottle = 3*dmax;
    
    // Limit minimum throttle to 0.8*dTo for VLL_z contorl
    if ( (minThrottle > 0.8*controlData.dTo) && (controlData.mode == VelocityControl) )
    {
        minThrottle = 0.8*controlData.dTo;
    }
}

template<typename TempType1, typename TempType2>
inline TempType2 mapToValue(TempType1 pwm, TempType1 pwmMin, TempType1 pwmMax, TempType2 valueMin, TempType2 valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = ( static_cast<double>(valueMax-valueMin) ) / ( static_cast<double>(pwmMax-pwmMin) );
    
    return limit( static_cast<TempType2>( slope*(pwm-pwmMin)+valueMin ) , valueMin, valueMax);
}

template double mapToValue(int, int, int, double, double);
template int mapToValue(double, double, double, int, int);

template<typename TempType>
inline TempType limit(TempType value, TempType valueMin, TempType valueMax)
{
    if (value < valueMin) { return valueMin; }
    if (value > valueMax) { return valueMax; }
    return value;
}

template int limit(int, int, int);
template double limit(double, double, double);

double getPwmCmd(channelType chn) { return double( pwmCmd[chn] ); }
