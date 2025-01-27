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

#ifndef SIMULATION
    #include "Servo.h"
#endif

ControlType controlData;

// Servos
Servo T1esc;
Servo T2esc;
Servo T3esc;
Servo T4esc;

// PWM In
int prevPwmCmd[nChannels];

// Navigation Data
double eulerAngles[3];
double velLL[3];
double position[3];
double accelZ;

// IMU Data
double bodyRates[3];

// Limits
int minPWM;
int maxPWM;
double minThrottle;
double maxThrottle;
double minRPM;
double maxRPM;

unsigned short altHoldLatchCounter;
unsigned short groundCount;

SensorErrorType accelError[3];
SensorErrorType gyroError[3];
const SensorErrorType* nav_gyroError;
const SensorErrorType* nav_accelError;

// Gains
// Position Channels
double wn_h = 0.5 * hz2rps;

double kp_h;

// Velocity Channels
double wn_vx   = 0.1 * hz2rps;
double zeta_vx = 2.0;
double wn_vy   = 0.1 * hz2rps;
double zeta_vy = 2.0;
double wn_vz   = 5.0 * hz2rps;
double zeta_vz = 1.0;

double kp_vx;
double ki_vx;
double kp_vy;
double ki_vy;
double kp_vz;
double ki_vz;

// Attitude Channels
double wn_roll    = 3.0 * hz2rps;
double zeta_roll  = 2.0;
double wn_pitch   = 3.0 * hz2rps;
double zeta_pitch = 2.0;
double wn_yaw     = 20.0 * hz2rps;

double kp_roll;
double kd_roll;
double kp_pitch;
double kd_pitch;
double kp_yaw;

bool controls_setup = false;

void FsControls_setup()
{
#ifdef CONTROLS
    // Servos
    T1esc.attach(T1PIN);
    T2esc.attach(T2PIN);
    T3esc.attach(T3PIN);
    T4esc.attach(T4PIN);
    
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    T1esc.writeMicroseconds(PWMMIN);
    
    // Data
    for (int i=0; i<3; i++)
    {
        eulerAngles[i] = 0.0;
        bodyRates[i] = 0.0;
        velLL[i] = 0.0;
    }
    accelZ = 0.0;

    nav_gyroError = 0;
    nav_accelError = 0;
    
    // Control Gains
    altHoldLatchCounter = 0;
    groundCount = 0;
    
    // Altitude Channel Gains
    kp_h = -wn_h;
    
    // Velocity Channel Gains
    ki_vx  = wn_vx/(KVX);
    kp_vx  = 2.0*zeta_vx*wn_vx/(KVX);
    ki_vy  = wn_vy/(KVY);
    kp_vy  = 2.0*zeta_vy*wn_vy/(KVY);
    kp_vz  = wn_vz/(KVZ);
    ki_vz  = 2.0*zeta_vz*wn_vz/(KVZ);
    
    // Attitude Channel Gains
    kp_roll  = wn_roll*wn_roll/(KROLL);
    kd_roll  = 2.0*zeta_roll*wn_roll/(KROLL);
    kp_pitch = wn_pitch*wn_pitch/(KPITCH);
    kd_pitch = 2.0*zeta_pitch*wn_pitch/(KPITCH);
    kp_yaw   = wn_yaw/(KYAW);
    
    // Limits
    minPWM = PWMMIN;
    maxPWM = PWMMINRPM;
    minRPM = 0.0;
    maxRPM = MINRPM;
    minThrottle = 0.0;
    maxThrottle = MINTHROTTLE;
    
    // PWM
    for (int iCh = THROTTLE_CHANNEL; iCh != nChannels; iCh++)
    {
        prevPwmCmd[iCh] = (PWMMIN+PWMMAX)/2.0;
    }
    controls_setup = true;
#endif
}

void FsControls_groundDetection()
{
#ifdef GROUND_DETECTION
    if (!FsImu_IMUGood()) { return; }
    
    if (nav_gyroError && nav_accelError)
    {
        double dGyro;
        double dAccel;
        controlData.movingDetection = false;
        for (int i=0; i<3; i++)
        {
            gyroError[i].compute();
            accelError[i].compute();
        
            dGyro = fabs(gyroError[i].std - nav_gyroError[i].std);
            dAccel = fabs(accelError[i].std - nav_accelError[i].std);
        
            //display(i); display(") ["); display(gyroError[i].std); display(", "); display(accelError[i].std); display("]\n");
            
            controlData.movingDetection = controlData.movingDetection | (dGyro > 0.1) | (dAccel > 0.1);
        }
    }
    
    if (controlData.onGround)
    {
        // Detect take off command
        if (!controlData.takeOff &&
            ((controlData.mode == ThrottleControl && controlData.pwmCmd[THROTTLE_CHANNEL] > PWMMINRPM) || controlData.VLLzCmd < -AltHoldVelExit) )
        {
            controlData.takeOff = true;
        }
        
        // Detect crash landings
        if (!controlData.takeOff &&
            (fabs(eulerAngles[0]) > 45.0*degree2radian || fabs(eulerAngles[1]) > 45.0*degree2radian) &&
            (sqrt(bodyRates[0]*bodyRates[0] + bodyRates[1]*bodyRates[1]) > 100*degree2radian))
        {
            controlData.crashLand = true;
            display("Crash landing detected! [roll, pitch, rate]: [");
            display(fabs(eulerAngles[0])*radian2degree); display(", ");
            display(fabs(eulerAngles[1])*radian2degree); display(", ");
            display(sqrt(bodyRates[0]*bodyRates[0] + bodyRates[1]*bodyRates[1])*radian2degree); display("]\n");
        }
        
        // Detect off ground
        if (controlData.takeOff && position[2] > 1.5)
        {
            controlData.onGround = false;
        }
    }
    // Check for ground contact
    else if (controlData.VLLzCmd >= 0 && (controlData.VLLzCmd - velLL[2]) > AltHoldVelEnter && controlData.dT == controlData.dTmin && fabs(accelZ) < 0.5 && fabs(velLL[2]) < 0.5)
    {
        groundCount++;
        if (groundCount > 50)
        {
            display("Fs_Constrols on ground. ");
            display("Accel: ");
            display(accelZ);
            display(" groundCount: ");
            display(groundCount);
            display("\n");
            controlData.onGround = true;
            controlData.takeOff  = false;
        }
    }
    else if (accelZ < -2.0*Gravity && velLL[2] > 0.5)
    {
        groundCount++;
        display("Ground count: "); display(groundCount); display(", "); display(accelZ); display("\n");
        if (groundCount > 1)
        {
            display("Fs_Constrols on ground. ");
            display("Accel: ");
            display(accelZ);
            display(" groundCount: ");
            display(groundCount);
            display("\n");
            
            controlData.onGround = true;
            controlData.takeOff  = false;
        }
    }
    else
    {
        groundCount = 0;
    }
#endif
}

void FsControls_performControls(double &ctrlDt)
{
#ifdef CONTROLS
    // Not initialization for controls
    if ( (controlData.mode == NoControl) ||
        (!controls_setup) ||
        (FsNavigation_getNavState() <= Calibration) ||
        (!FsImu_IMUGood())
        )
    {
        return;
    }
    
    performControls(ctrlDt);
    
    setMotors();
    
    controlData.timestamp = getTime();
#endif
}

void performControls(double &ctrlDt)
{
    // Discritize Commands to Limit Unintentional Change
    for (int iCh = THROTTLE_CHANNEL; iCh != nChannels; iCh++)
    {
        // TODO - check, this may be a bug! Maybe move prevPwmCmd after discretize function
        prevPwmCmd[iCh] = controlData.pwmCmd[iCh];
        controlData.pwmCmd[iCh] = discretize(controlData.pwmCmd[iCh], prevPwmCmd[iCh], PWMMIN, PWMMAX, minPWMIncr);
    }
    
    // Compute Throttle PWM Limits
    rpmPwmLimits( (float) controlData.pwmCmd[THROTTLE_CHANNEL], (float) PWMMINRPM );
    
    // Convert PWM to values
    if (controlData.mode == AttitudeControl)
    {
        controlData.VLLzCmd    = -mapToValue(controlData.pwmCmd[THROTTLE_CHANNEL], (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.rollCmd    = mapToValue(controlData.pwmCmd[ROLL_CHANNEL]     , (int) PWMMIN, (int) PWMMAX, (double) -MAXROLL   , (double) MAXROLL);    // Roll (rad)
        controlData.pitchCmd   = mapToValue(controlData.pwmCmd[PITCH_CHANNEL]    , (int) PWMMIN, (int) PWMMAX, (double) -MAXPITCH  , (double) MAXPITCH);   // Pitch (rad)
        controlData.yawRateCmd = mapToValue(controlData.pwmCmd[YAW_CHANNEL]      , (int) PWMMIN, (int) PWMMAX, (double) -MAXYAWRATE, (double) MAXYAWRATE); // Yaw Rate (rad/s)
    }
    
    else if (controlData.mode == VelocityControl)
    {
        // Nav VN std < 0.1 m/s and Nav VE std < 0.1 m/s
        controlData.VLLzCmd    = -mapToValue(controlData.pwmCmd[THROTTLE_CHANNEL], (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.VLLyCmd    = mapToValue(controlData.pwmCmd[ROLL_CHANNEL]    , (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.VLLxCmd    = mapToValue(controlData.pwmCmd[PITCH_CHANNEL]   , (int) PWMMIN, (int) PWMMAX, (double) -MAXVELOCITY, (double) MAXVELOCITY); // (m/s)
        controlData.yawRateCmd = mapToValue(controlData.pwmCmd[YAW_CHANNEL]     , (int) PWMMIN, (int) PWMMAX, (double) -MAXYAWRATE , (double) MAXYAWRATE);  // (rad/s)
        
        // Outer Velocity Loop to Generate Attitude Commands
        controlData.rollCmd  = kp_vy*(controlData.VLLyCmd - velLL[1]) + ki_vy*controlData.vy_int_error;
        controlData.pitchCmd = kp_vx*(controlData.VLLxCmd - velLL[0]) + ki_vx*controlData.vx_int_error;
        
        // Limit Commands
        controlData.rollCmd  = limit(controlData.rollCmd, (double) -MAXROLL, (double) MAXROLL);
        controlData.pitchCmd = limit(controlData.pitchCmd, (double) -MAXPITCH, (double) MAXPITCH);
    }
    
    // Control Attitude
    if (controlData.crashLand)
    {
        controlData.dT = 0.0;
        controlData.da = 0.0;
        controlData.de = 0.0;
        controlData.dr = 0.0;
        minThrottle = 0.0;
    }
    else if (controlData.onGround && !controlData.takeOff)
    {
        controlData.dT = dMIN;
        controlData.da = 0.0;
        controlData.de = 0.0;
        controlData.dr = 0.0;
        
        controlData.vx_int_error = 0.0;
        controlData.vy_int_error = 0.0;
        controlData.vz_int_error = 0.0;
    }
    else if (controlData.mode == ThrottleControl)
    {
        controlData.dT = mapToValue(controlData.pwmCmd[THROTTLE_CHANNEL], minPWM, maxPWM, minThrottle, maxThrottle);
        controlData.da = 0.0;
        controlData.de = 0.0;
        controlData.dr = 0.0;
    }
    else
    {
        verticalModing();
        
        if (controlData.controlAltitude)
        {
            controlData.VLLzCmd = kp_h*(controlData.hCmd - position[2]);
            controlData.VLLzCmd  = limit(controlData.VLLzCmd, (double) -MAXVELOCITY, (double) MAXVELOCITY);
        }
        
        controlData.dT = kp_vz*(controlData.VLLzCmd - velLL[2])  + ki_vz*controlData.vz_int_error + dTo;
        controlData.da = kp_roll *(controlData.rollCmd    - eulerAngles[0]) - kd_roll*bodyRates[0];
        controlData.de = kp_pitch*(controlData.pitchCmd   - eulerAngles[1]) - kd_pitch*bodyRates[1];
        controlData.dr = kp_yaw  *(controlData.yawRateCmd - bodyRates[2]);

        // Increase throttle to allow attitude following
        minAttitudeThrottle();
        
        // Integral Errors
        if (controlData.pitchCmd > -MAXPITCH && controlData.pitchCmd < MAXPITCH && controlData.mode == VelocityControl)
        {
            controlData.vx_int_error += (controlData.VLLxCmd - velLL[0])*ctrlDt;
        }
        else if (controlData.mode != VelocityControl)
        {
            controlData.vx_int_error = 0.0;
        }
        
        if (controlData.rollCmd > -MAXROLL && controlData.rollCmd < MAXROLL && controlData.mode == VelocityControl)
        {
            controlData.vy_int_error += (controlData.VLLyCmd - velLL[1])*ctrlDt;
        }
        else if (controlData.mode != VelocityControl)
        {
            controlData.vy_int_error = 0.0;
        }
        
        if (controlData.dT > minThrottle && controlData.dT < maxThrottle)
        {
            controlData.vz_int_error += (controlData.VLLzCmd - velLL[2])*ctrlDt;
        }
    }
    
    // Limit Commands
    controlData.dT = limit(controlData.dT, minThrottle, maxThrottle);
    
    controlData.daRaw = controlData.da;
    controlData.deRaw = controlData.de;
    controlData.drRaw = controlData.dr;
    
    // Prevent RPM's less than RPMMIN
    controlData.da = limit(controlData.da, -controlData.dT/3.0, controlData.dT/3.0);
    controlData.de = limit(controlData.de, -controlData.dT/3.0, controlData.dT/3.0);
    controlData.dr = limit(controlData.dr, -controlData.dT/3.0, controlData.dT/3.0);
    
    // Prevent RPM's greater than RPMMAX
    controlData.da = limit(controlData.da, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    controlData.de = limit(controlData.de, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    controlData.dr = limit(controlData.dr, -(dMAX-controlData.dT)/3.0, (dMAX-controlData.dT)/3.0);
    
    controlData.dTmin = minThrottle;
    controlData.dTmax = maxThrottle;
    controlData.minCtrl = controlData.dT/3.0;
    controlData.maxCtrl = (dMAX-controlData.dT)/3.0;
}

void verticalModing()
{
    if (controlData.mode == Autopilot) { return; }
    
    if (!controlData.controlAltitude && fabs(controlData.VLLzCmd) < AltHoldVelEnter)
    {
        altHoldLatchCounter++;
        if (altHoldLatchCounter > 10)
        {
            altHoldLatchCounter = 0;
            controlData.controlAltitude = true;
            controlData.hCmd = position[2];
        }
    }
    else if (controlData.controlAltitude && fabs(controlData.VLLzCmd) > AltHoldVelExit)
    {
        altHoldLatchCounter++;
        if (altHoldLatchCounter > 10)
        {
            altHoldLatchCounter = 0;
            controlData.controlAltitude = false;
        }
    }
    else
    {
        altHoldLatchCounter = 0;
    }
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

void FsControls_setMode(ControlMode mode)
{
    if (mode != controlData.mode)
    {
        display("FsControls mode change. ");
        display((int) controlData.mode);
        display(" -> ");
        display((int) mode);
        display("\n");
    }
    controlData.mode = mode;
    
}

void FsControls_setPWMCommands(const int* pwmIn)
{
    for (int iCh = THROTTLE_CHANNEL; iCh != nChannels; iCh++)
    {
        controlData.pwmCmd[iCh] = pwmIn[iCh];
    }
}

#ifdef SIMULATION
void FsControls_setSimulationModels(ModelMap* pMap)
{
    T1esc.servo_setSimulationModels(pMap);
    T2esc.servo_setSimulationModels(pMap);
    T3esc.servo_setSimulationModels(pMap);
    T4esc.servo_setSimulationModels(pMap);
}
#endif

void FsControls_setControlsData(const IMUtype* pIMUdataIn, const NavType* pNavDataIn)
{
    for (int i=0; i<3; i++)
    {
        bodyRates[i]   = pIMUdataIn->gyro[i] - pNavDataIn->gyroBias[i];
        eulerAngles[i] = pNavDataIn->eulerAngles[i];
        position[i]    = pNavDataIn->position[i];
        
        gyroError[i].update(pIMUdataIn->gyro[i]);
        accelError[i].update(pIMUdataIn->accel[i]);
    }
    FsNavigation_NEDToLL(velLL, pNavDataIn->velNED);
    accelZ = pNavDataIn->accelBody[2];
}

void FsControls_setIMUStatistics(const SensorErrorType* calGyroError, const SensorErrorType* calAccelError)
{
    if (nav_gyroError && nav_accelError) { return; }
    nav_gyroError = calGyroError;
    nav_accelError = calAccelError;
}

void FsControls_resetMovingDetection()
{
    for (int i=0; i<3; i++)
    {
        gyroError[i].reset();
        accelError[i].reset();
    }
}

const ControlType* FsControls_getControlData()
{
    return &controlData;
}

bool FsControls_onGround()
{
    return controlData.onGround;
}

bool FsControls_movingDetection()
{
    return controlData.movingDetection;
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
    /*if (value < threshold)
    {
        minPWM = PWMMIN;
        maxPWM = PWMMINRPM;
        minRPM = 0.0;
        maxRPM = MINRPM;
        minThrottle = 0.0;
        maxThrottle = MINTHROTTLE;
    }
    else
    {*/
        minPWM = PWMMINRPM;
        maxPWM = PWMMAX;
        minRPM = MINRPM;
        maxRPM = MAXRPM;
        minThrottle = MINTHROTTLE;
        maxThrottle = MAXTHROTTLE;
    //}
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
    if (minThrottle > 0.8*dTo)
    {
        minThrottle = 0.8*dTo;
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
