//
//  InitialConditions.h
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/9/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef InitialConditions_h
#define InitialConditions_h

#include <math.h>
#include <chrono>
#include <stdio.h>
#include <map>
#include <iomanip>
#include <fstream>

#include "aircraft_simulation_types.h"
#include "utilities.hpp"

//#define RealTime

// Time
const double runTime_init          = 60.0*10.0;
const double printInterval_init    = 5.0;
const double saveInterval_init     = 1.0;
const double dynamicsInterval_init = 1.0/800.0;
const double clock_dt              = 1.0/800.0;//1.0/19200.0; // time interval when not in real time mode

const int month_init  = EARTHCONSTANTS::SEP;
const int day_init    = 30;
const int year_init   = 2022;
const int hour_init   = 13;
const int minute_init = 30;
const int second_init = 00;

// Print options
const bool saveOutput  = true;
const bool printOutput = true;
const bool plotOutput  = false;
const std::string savefile = "output_moving_Qt.csv";
const std::string gps_file = "/Users/alexandermclean/Documents/Cpp-Aircraft-Simulation/gps_nav_messages_Adeline4.txt";
//const std::string gps_file = "/Users/alexandermclean/Documents/Cpp-Aircraft-Simulation/gps_messages.txt";
const bool readGPSFile = false;

// Initial States
const double velNED_init[3]  = {0, 0, 0};
const double posLLH_init[3] = {28.5997222, -81.3394444, 25+1.968}; // deg, deg, ft
const double eulerAngles_init[3] = {0, 0.0, 45.0}; // {0, 6.507, 0};
const double bodyRates_init[3]  = {0, 0, 0};
const double actuators_init[4]   = {0.0, 0.0, 0.0, 0.0}; //{de,da,dr,dT}
const int inputMode_init = 0; // 0 - external, 1 - keyboard, 2 - table

// Trim
// Uses initial position above
// Uses initial heading from above
const bool trim = false;

static SpeedType<double> trimSpeed(0.0,metersPerSecond); // Ground speed
static AngleType<double> trimRoll(0.0, degrees);         // Roll/bank angle
static AngleType<double> trimGamma(0.0, degrees);        // Climb angle

// Aircraft Constants
/*
const double mass_init = 0.372;// Weight = 4.2 N = 1 lb
const double inertia_init[3][3] = {
    {0.00081 , 0.0    , -0.00114 },
    {0.0     , 0.00602, 0.0      },
    {-0.00114, 0.0    , 0.00643  }};
*/
// Quadcopter Constants
const double mass_init = 0.5;// Weight = 4.9 N
const double inertia_init[3][3] = {
    {0.0196 , 0.0      , 0.0     },
    {0.0       , 0.0196, 0.0     },
    {0.0       , 0.0   , 0.0264  }};

// Reference Frames
const double imuFrame_init[3] = {0, 0, 0};

// Test Modes
const bool testRotations = false; // Test rotation matrices given known vectors and rotations
const bool testDynamics  = false; // Generate desired dynamics and test propogation from inital conditions
const bool testGround    = false; // Generate desired body forces for ground to react to
const double testForceDynamics[3]  = {0, 0, 0};
const double testMomentDynamics[3] = {0, 0, 0};
const double testForceGround[3]  = {0.0, 0.0, 0};

const double zero_init[3]         = {0, 0, 0};
const double quaternion_init[4]   = {1, 0, 0, 0};
const double quaternion_init2[4]  = {0, 0, 0, 0};
const double identityMatrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// World constants
const double elevation_init = 7.62;
const double Rearth = 6371e+3;
const double GM     = 3.9857e+14;
const double zeroTolerance = 0.01;
const double Vref = 10.0;

// Quadcopter Geometry
const double quadTopArea = 0.16;

// Aircraft Geometry
const double span  = 0.52;
const double meanChord = 0.13;
const double wingArea = span*meanChord;
const double AR = span/meanChord;
const double e  = 0.8;

// Stop Conditions
const double maxPitch = 95;
const double minPitch = -95;
const double maxGroundPitch = 85;
const double minGroundPitch = -45;
const double maxGroundRoll = 45;
const double minGroundRoll = -45;
const double minAlpha = -250;
const double maxAlpha = 250;

// Typedefs
#ifdef RealTime
    typedef std::chrono::time_point<std::chrono::system_clock> systemTime;
    typedef std::chrono::duration<double> timeDuration;
#else
    typedef double systemTime;
    typedef double timeDuration;
#endif

#endif // InitialConditions_h
