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
enum ControlMode {ThrottleControl, AttitudeControl, VelocityControl, Autopilot, NoControl};

struct ControlType {
    // Command Input
    int pwmCmd[nChannels];
    
    // Commands
    double hCmd;
    double VLLxCmd;
    double VLLyCmd;
    double VLLzCmd;
    double rollCmd;
    double pitchCmd;
    double yawRateCmd;
    
    // Errors
    double vx_int_error;
    double vy_int_error;
    double vz_int_error;
    
    // roll/pitch/yaw/throttle motor commands
    double da;
    double de;
    double dr;
    double dT;
    
    double daRaw;
    double deRaw;
    double drRaw;
    
    // limits
    double dTmin;
    double dTmax;
    double minCtrl;
    double maxCtrl;
    
    // RPM
    double rpmSq[4];
    
    // PWM
    double TPWM[4];
    
    // Ground Detection
    bool onGround;
    bool takeOff;
    bool crashLand;
    bool movingDetection;
    
    bool controlAltitude;
    ControlMode mode;
    double timestamp;
    
    ControlType()
    {
        hCmd     = 0.0;
        VLLxCmd  = 0.0;
        VLLyCmd  = 0.0;
        VLLzCmd  = 0.0;
        rollCmd  = 0.0;
        pitchCmd = 0.0;
        yawRateCmd = 0.0;
        
        vx_int_error = 0.0;
        vy_int_error = 0.0;
        vz_int_error = 0.0;
        
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
        takeOff    = false;
        crashLand  = false;
        onGround   = true;
        movingDetection = false;
        
        controlAltitude = false;
        mode = NoControl;
        timestamp = 0.0;
    }
};

// External Access
void FsControls_setup();
void FsControls_groundDetection();
void FsControls_performControls(double &ctrlDt);
void FsControls_resetMovingDetection();

// Setters
void FsControls_setMode(ControlMode mode);
void FsControls_setPWMCommands(const int* pwmIn);
void FsControls_setControlsData(const IMUtype* pIMUdataIn, const NavType* pNavDataIn);
void FsControls_setIMUStatistics(const SensorErrorType* calGyroError, const SensorErrorType* calAccelError);

#ifdef SIMULATION
    void FsControls_setSimulationModels(ModelMap* pMap);
#endif

// Getters
const ControlType* FsControls_getControlData();
bool               FsControls_onGround();
bool               FsControls_movingDetection();
// Internal Access
void discretizeCommands();
int discretize(int desiredCommand, int command, int minCmd, int maxCmd, int deltaPwm);
void performControls(double &ctrlDt);
void verticalModing();
void setMotors();

// Limits
void rpmPwmLimits(float value, float threshold);
void minAttitudeThrottle();

template<typename TempType1, typename TempType2>
inline TempType2 mapToValue(TempType1 pwm, TempType1 pwmMin, TempType1 pwmMax, TempType2 valueMin, TempType2 valueMax);

template<typename TempType>
inline TempType limit(TempType value, TempType valueMin, TempType valueMax);
#endif /* fs_controls_hpp */
