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
    typedef std::string String;
#endif

// Forward References
struct IMUtype;
struct NavType;
struct ControlType;
class ModelMap;

// Types
enum channelType {THROTTLE, ROLL, PITCH, YAWRATE, nChannels};

// Constants
const double deg2rad = M_PI/180.0;
const double RE = 6371e+3;
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

#define PWMMIN  1000
#define PWMMAX  2000

#define MAXROLL      20 * deg2rad // rad
#define MAXPITCH     20 * deg2rad // rad
#define MAXYAWRATE   45 * deg2rad // rad/s
#define MAXTHROTTLE  85 //(PWMMAX-PWMMIN)*1.85 // PWM

// GPS Pins
#define GPSintPIN = 3;

// Mapping
double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax);
unsigned long mapToPwm(double value, double valueMin, double valueMax, unsigned long pwmMin, unsigned long pwmMax);
unsigned long limit(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax);

// Time
double getTime(void);

// Printing
template<typename TempType>
void display(TempType val);

// LED
void LEDon();
void LEDoff();

void FsCommon_setSimulationModels(ModelMap* pMap);

#endif /* fs_common_hpp */
