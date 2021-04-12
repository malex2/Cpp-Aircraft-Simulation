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

ControlType controlData;

// Servos
Servo T1esc;
Servo T2esc;
Servo T3esc;
Servo T4esc;

// Attittude Commands
DiscreteCommand rollCmd;
DiscreteCommand pitchCmd;
DiscreteCommand yawRateCmd;

double rollCmdIn;
double pitchCmdIn;
double yawRateCmdIn;

// Command Constraints
double minDegree; // minimum change in command degree
double minDps;    // minimum change in command rate
double dPWMmax;

// IMU data
IMUtype *controls_pIMUdata = 0;
NavType *controls_pNavdata = 0;

bool controls_setup = false;

void FsControls_setup()
{
    // Variables
    minDegree = 0.5;
    minDps    = 1.0;
    dPWMmax   = 0.05*(PWMMAX - PWMMIN);
    
    rollCmdIn = 0.0;
    pitchCmdIn = 0.0;
    yawRateCmdIn = 0.0;
    
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
    yawRateCmd.setup(minDps*deg2rad, -MAXYAWRATE, MAXYAWRATE);
    
    controls_setup = true;
}

void FsControls_performControls()
{
    if ( (controlData.mode == NoControl) || (!controls_setup) ) { return; }
    
    discretizeCommands();
    
    performControls();
    
    setMotors();
}

void discretizeCommands()
{
    controlData.rollCmd    = rollCmd.discretize   ( rollCmdIn);
    controlData.pitchCmd   = pitchCmd.discretize  ( pitchCmdIn );
    controlData.yawRateCmd = yawRateCmd.discretize( yawRateCmdIn );
}

void performControls()
{
    if (controlData.mode == ThrottleControl)
    {
        
    }
    else if (controlData.mode == AttitudeControl)
    {
        controlData.dRoll  = 0.0;
        controlData.dPitch = 0.0;
        controlData.dYawRate = 0.0;
    }
}

void setMotors()
{
    // Shorthand for controlData
    ControlType* ctrl = &controlData;
    
    // Convert commands to PWM
    ctrl->rollPWM  = mapToPwm(ctrl->dRoll  , -MAXROLL, MAXROLL, -dPWMmax, dPWMmax);
    ctrl->pitchPWM = mapToPwm(ctrl->dPitch, -MAXPITCH, MAXPITCH, -dPWMmax, dPWMmax);
    ctrl->yawPWM   = mapToPwm(ctrl->dYawRate, -MAXYAWRATE, MAXYAWRATE, -dPWMmax, dPWMmax);
    ctrl->throttlePWM = mapToPwm(ctrl->throttleCmd, 0, 100, PWMMIN, PWMMAX);
    
    // Limit PWM
    ctrl->T1PWM = limit(ctrl->throttlePWM - ctrl->rollPWM + ctrl->pitchPWM + ctrl->yawPWM, PWMMIN, PWMMAX);
    ctrl->T2PWM = limit(ctrl->throttlePWM + ctrl->rollPWM + ctrl->pitchPWM - ctrl->yawPWM, PWMMIN, PWMMAX);
    ctrl->T3PWM = limit(ctrl->throttlePWM + ctrl->rollPWM - ctrl->pitchPWM + ctrl->yawPWM, PWMMIN, PWMMAX);
    ctrl->T4PWM = limit(ctrl->throttlePWM - ctrl->rollPWM - ctrl->pitchPWM - ctrl->yawPWM, PWMMIN, PWMMAX);
    
    // Write to servos
    T1esc.writeMicroseconds( static_cast<long> (ctrl->T1PWM) );
    T2esc.writeMicroseconds( static_cast<long> (ctrl->T2PWM) );
    T3esc.writeMicroseconds( static_cast<long> (ctrl->T3PWM) );
    T4esc.writeMicroseconds( static_cast<long> (ctrl->T4PWM) );
}

void FsControls_setMode(ControlMode mode) { controlData.mode = mode; }
void FsControls_setThrottleCmd(double throttleIn) { controlData.throttleCmd = throttleIn; }
void FsControls_setRollCmd(double rollIn) { rollCmdIn = rollIn; }
void FsControls_setPitchCmd(double pitchIn) { pitchCmdIn = pitchIn; }
void FsControls_setBodyYawRateCmd(double yawRateIn) { yawRateCmdIn = yawRateIn; }
void FsControls_setSimulationModels(ModelMap* pMap)
{
#ifdef SIMULATION
    T1esc.servo_setSimulationModels(pMap);
    T2esc.servo_setSimulationModels(pMap);
    T3esc.servo_setSimulationModels(pMap);
    T4esc.servo_setSimulationModels(pMap);
#endif
}

void FsControls_setIMUdata(IMUtype* pIMUdataIn)
{
    controls_pIMUdata = pIMUdataIn;
}

void FsControls_setNavdata(NavType* pNavDataIn)
{
    controls_pNavdata = pNavDataIn;
}

ControlType* FsControls_getControlData()
{
    controlData.timestamp = getTime();
    return &controlData;
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
    command = minCmd;
}

double DiscreteCommand::discretize(double desiredCommand)
{
    if (minCmd == maxCmd)
    {
        display("Warning: command not setup!\n");
        return command;
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
    
    return command;
}

void DiscreteCommand::updateCommandOption(int index)
{
    commandOption = minCmd + minIncr*index;
}
