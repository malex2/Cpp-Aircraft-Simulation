//
//  fs_controls.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_controls_hpp
#define fs_controls_hpp

#include "fs_common.hpp"

// Types
enum ControlMode {ThrottleControl, AttitudeControl, NoControl};

struct SignalErrorType {
    double error;
    double dError;
    double iError;
};

struct ControlType {
    // Command Input
    double rollCmd;
    double pitchCmd;
    double yawRateCmd;
    double throttleCmd;
    
    // roll/pitch/yaw motor commands
    double dRoll;
    double dPitch;
    double dYawRate;
    
    // roll/pitch/yaw motor PWM commands
    double rollPWM;
    double pitchPWM;
    double yawPWM;
    double throttlePWM;
    
    SignalErrorType rollError;
    SignalErrorType pitchError;
    SignalErrorType yawRateError;
    
    double T1PWM;
    double T2PWM;
    double T3PWM;
    double T4PWM;
    
    ControlMode mode;
    double timestamp;
    
    ControlType()
    {
        rollCmd = 0.0;
        pitchCmd = 0.0;
        yawRateCmd = 0.0;
        throttleCmd = 0.0;
        
        dRoll = 0.0;
        dPitch = 0.0;
        dYawRate = 0.0;
        
        rollPWM  = 0;
        pitchPWM = 0;
        yawPWM   = 0;
        throttlePWM = PWMMIN;
        
        T1PWM = 0;
        T2PWM = 0;
        T3PWM = 0;
        T4PWM = 0;
        
        mode = NoControl;
    }
};

// External Access
void FsControls_setup();
void FsControls_performControls();

// Setters
void FsControls_setMode(ControlMode mode);
void FsControls_setThrottleCmd(double throttleIn);
void FsControls_setRollCmd(double rollIn);
void FsControls_setPitchCmd(double pitchIn);
void FsControls_setBodyYawRateCmd(double yawRateIn);
void FsControls_setSimulationModels(ModelMap* pMap);

void FsControls_setIMUdata(IMUtype* pIMUdataIn);
void FsControls_setNavdata(NavType* pNavDataIn);

// Getters
ControlType* FsControls_getControlData();

// Internal Access
void discretizeCommands();
void performControls();
void setMotors();

class DiscreteCommand {
public:
    DiscreteCommand();
    
    void setup(double minIncr, double minCmd, double maxCmd);
    double discretize(double desiredCommand);
private:
    double minCmd;
    double maxCmd;
    double minIncr;
    
    double command;
    double commandOption;
    bool increasing;
    int lengthArray;
    
    void updateCommandOption(int index);
};

#endif /* fs_controls_hpp */
