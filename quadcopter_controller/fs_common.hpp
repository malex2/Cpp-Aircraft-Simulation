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
//#define SIMULATION

#ifdef SIMULATION
    #include "model_mapping.hpp"
    #include "arduino_class_models.hpp"
    #include <iostream>
    #include <iomanip>
    #include <fstream>
    typedef std::string String;
#else
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
enum channelType {THROTTLE, ROLL, PITCH, YAW, nChannels};

// Constants
#define degree2radian M_PI/180.0
#define radian2degree 180.0/M_PI
#define RE  6371e+3
#define GMe 3.9857e+14

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
#define MAXROLL      20.0 * degree2radian // rad
#define MAXPITCH     20.0 * degree2radian // rad
#define MAXYAWRATE   360.0 * degree2radian // rad/s
#define MINTHROTTLE  dMIN
#define MAXTHROTTLE  0.85*dMAX

#define minDeg 0.5
#define minDps 1.0
#define minPWMΙncr 5.0

#define quadMass 0.5

// GPS Pins
#define GPSRXPIN 2
#define GPSTXPIN 3

// Time
double getTime();

// Printing
template<typename TempType>
void display(TempType val);

// LED
void LEDon();
void LEDoff();

void FsCommon_setSimulationModels(ModelMap* pMap);

#endif /* fs_common_hpp */
