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
enum ControlMode {ThrottleControl, AttitudeControl, VelocityControl, NoControl};

struct ControlType {
    // Command Input
    double VLLxCmd;
    double VLLyCmd;
    double VLLzCmd;
    double rollCmd;
    double pitchCmd;
    double yawRateCmd;
    
    // roll/pitch/yaw motor commands
    double da;
    double de;
    double dr;
    double dT;
    double dTo;
    
    //
    double rpmSq[4];
    
    // PWM
    double TPWM[4];
    
    double timestamp;
    ControlMode mode;
    
    ControlType()
    {
        VLLxCmd = 0.0;
        VLLyCmd = 0.0;
        VLLzCmd = 0.0;
        rollCmd = 0.0;
        pitchCmd = 0.0;
        yawRateCmd = 0.0;
        
        da = 0.0;
        de = 0.0;
        dr = 0.0;
        dT = 0.0;
        dTo = 1.2*4.0*MINRPM*MINRPM;
        
        for (int i=0; i<3; i++)
        {
            TPWM[i]  = 0.0;
            rpmSq[i] = 0.0;
        }
        
        timestamp = 0.0;
        mode = NoControl;
    }
};

// External Access
void FsControls_setup();
void FsControls_performControls();

// Setters
void FsControls_setMode(ControlMode mode);
void FsControls_setPWMCommands(int* pwmIn);
void FsControls_setSimulationModels(ModelMap* pMap);

void FsControls_setControlsData(IMUtype* pIMUdataIn, NavType* pNavDataIn);

// Getters
ControlType* FsControls_getControlData();
double getPwmCmd(channelType chn);

// Internal Access
void discretizeCommands();
int discretize(int desiredCommand, int command, int minCmd, int maxCmd, int deltaPwm);
void performControls();
void setMotors();

// Limits
void rpmPwmLimits(float value, float threshold);
void minAttitudeThrottle();

template<typename TempType1, typename TempType2>
inline TempType2 mapToValue(TempType1 pwm, TempType1 pwmMin, TempType1 pwmMax, TempType2 valueMin, TempType2 valueMax);

template<typename TempType>
inline TempType limit(TempType value, TempType valueMin, TempType valueMax);
#endif /* fs_controls_hpp */
