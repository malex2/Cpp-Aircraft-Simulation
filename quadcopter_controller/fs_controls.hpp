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
    int pwmCmd[nChannels];
    
    double VLLxCmd;
    double VLLyCmd;
    double VLLzCmd;
    double rollCmd;
    double pitchCmd;
    double yawRateCmd;
    
    // roll/pitch/yaw/throttle motor commands
    double da;
    double de;
    double dr;
    double dT;
    
    // limits
    double dTmin;
    double dTmax;
    double minCtrl;
    double maxCtrl;
    
    // RPM
    double rpmSq[4];
    
    // PWM
    double TPWM[4];
    
    ControlMode mode;
    double timestamp;
    
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
        
        dTmin = 0.0;
        dTmax = 0.0;
        minCtrl = 0.0;
        maxCtrl = 0.0;
        
        for (int i=0; i<4; i++)
        {
            pwmCmd[i] = (PWMMIN+PWMMAX)/2.0;
            TPWM[i]  = PWMMIN;
            rpmSq[i] = 0.0;
        }
        mode = NoControl;
        timestamp = 0.0;
    }
};

// External Access
void FsControls_setup();
void FsControls_performControls();

// Setters
void FsControls_setMode(ControlMode mode);
void FsControls_setPWMCommands(int* pwmIn);
void FsControls_setControlsData(IMUtype* pIMUdataIn, NavType* pNavDataIn);
#ifdef SIMULATION
    void FsControls_setSimulationModels(ModelMap* pMap);
#endif

// Getters
ControlType* FsControls_getControlData();

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
