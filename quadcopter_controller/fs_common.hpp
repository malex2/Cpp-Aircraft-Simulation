//
//  fs_common.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_common_hpp
#define fs_common_hpp

#include <stdio.h>
#include <math.h>

// MACROS
#define SIMULATION

#ifdef SIMULATION
    #include "model_mapping.hpp"
    #include "arduino_class_models.hpp"
    #include <iostream>
    #include <iomanip>
    #include <fstream>
    typedef std::string String;

#else
    #include "Wire.h"
    #include "arduino.h"
    #ifdef max
        #undef max
        #undef min
    #endif
    #define LEDPIN LED_BUILTIN
#endif

// Forward References
struct IMUtype;
struct NavType;
struct ControlType;
class ModelMap;
class Utilities;

// Types
enum channelType {THROTTLE_CHANNEL, ROLL_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, nChannels};

// Constants
#define degree2radian M_PI/180.0
#define radian2degree 180.0/M_PI
#define RE  6371e+3
#define GMe 3.9857e+14
#define Gravity 9.80665

// Pulse In Pins
#define THROTTLEPIN  7  // CH3
#define ROLLPIN      8  // CH4
#define PITCHPIN     12 // CH5
#define YAWPIN       13 // CH6

// Pulse Out Pins
#define T1PIN  6
#define T2PIN  9
#define T3PIN  10
#define T4PIN  11

// GPS Pins
#define GPSRXPIN 2
#define GPSTXPIN 3

// IMU Pins
#define IMUSDAPIN A4
#define IMUSCLPIN A5

// PWM
#define PWMMIN 1000
#define PWMMAX 2000

#define PWMMINRPM 1100
#define MINRPM 500
#define MAXRPM 6440
#define dMIN   4.0*MINRPM*MINRPM
#define dMAX   4.0*MAXRPM*MAXRPM

// Min/Max Limits
#define MAXVELOCITY  1.5 // m/s - 5 ft/s
#define MAXROLL      25.0 * degree2radian // rad
#define MAXPITCH     25.0 * degree2radian // rad
#define MAXYAWRATE   360.0 * degree2radian // rad/s
#define MINTHROTTLE  dMIN
#define MAXTHROTTLE  0.85*dMAX

// IMU Addresses
#define MPU_ADDR   0x68
#define PWR_MGMT_1 0x6B // PWR_MGMT_1 register
#define GYRO_REG   0x1B // Gryo register
#define ACC_REG    0x1C // Accelerometer register
#define ACC_OUT    0x3B // (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
#define INT        0x37
#define INT_ENABLE 0x38

// Baramoter Addresses
#define BMP180_ADDR                0x77 // 7-bit address
#define BMP180_REG_CONTROL         0xF4
#define BMP180_REG_RESULT          0xF6
#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0   0x34
#define BMP180_COMMAND_PRESSURE1   0x74
#define BMP180_COMMAND_PRESSURE2   0xB4
#define BMP180_COMMAND_PRESSURE3   0xF4
#define AC1_ADDR  0xAA
#define AC2_ADDR  0xAC
#define AC3_ADDR  0xAE
#define AC4_ADDR  0xB0
#define AC5_ADDR  0xB2
#define AC6_ADDR  0xB4
#define VB1_ADDR  0xB6
#define VB2_ADDR  0xB8
#define MB_ADDR   0xBA
#define MC_ADDR   0xBC
#define MD_ADDR   0xBE

#define minDeg 0.5
#define minDps 1.0
#define minPWMIncr 5.0
#define quadMass 0.5

#define highRate 5.0
#define highAccel 30.0
#define zeroAngle 30.0*degree2radian

// Time
double getTime();

// Errors
double errorToVariance(double maxError);

// Printing
template<typename TempType>
void display(TempType val);

// LED
void LEDon();
void LEDoff();

#ifdef SIMULATION
    void FsCommon_setSimulationModels(ModelMap* pMap);
#endif

#endif /* fs_common_hpp */
