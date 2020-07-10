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

#define RealTime

// Time
const float runTime_init          = 300;
const float printInterval_init    = 1;
const float saveInterval_init     = 0.1;
const float dynamicsInterval_init = 0.001;
const float clock_dt              = 0.001; // time interval when not in real time mode

// Print options
const bool saveOutput  = true;
const bool printOutput = true;
const bool plotOutput  = false;
const std::string savefile = "output.csv";

// Initial States
const float velNED_init[3]  = {0, 0, 0};
const float posBody_init[3] = {28.5997222, -81.3394444, 0.3}; // deg, deg, ft
const float eulerAngles_init[3] = {0, 6.507, 0}; // 6.507
const float eulerRates_init[3]  = {0, 0, 0};
const float actuators_init[4]   = {0.0, 0.0, 0.0, 0.0}; //{de,da,dr,dT}
const inputModeType inputMode_init = keyboard;

// Aircraft Constants
const float mass_init = 0.372;// Weight = 4.2 N = 1 lb
const float inertia_init[3][3] = {
    {0.00081 , 0.0    , -0.00114 },
    {0.0     , 0.00602, 0.0      },
    {-0.00114, 0.0    , 0.00643  }};

/*
const float inertia_init[3][3] = {
    {3  , 0.0, 0},
    {0.0  , 3, 0.0  },
    {0, 0.0, 3  }};
*/
// Reference Frames
const float imuFrame_init[3] = {0, 0, 0};

// Test Modes
const bool testRotations = false; // Test rotation matrices given known vectors and rotations
const bool testDynamics  = false; // Generate desired dynamics and test propogation from inital conditions
const bool testGround    = false; // Generate desired body forces for ground to react to
const float testForceDynamics[3]  = {0, 0, 0};
const float testMomentDynamics[3] = {0, 0, 0};
const float testForceGround[3]  = {0.0, 0.0, 0};

const float zero_init[3] = {0, 0, 0};
const float quaternion_init[4] = {1, 0, 0, 0};
const float quaternion_init2[4] = {0, 0, 0, 0};
const float identityMatrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// World constants
const float elevation_init = 0;
const float Rearth = 6371e+3;
const float GM     = 3.9857e+14;
const float zeroTolerance = 0.01;

// Aircraft Geometry
const float span  = 0.52;
const float meanChord = 0.13;
const float wingArea = span*meanChord;
const float AR = span/meanChord;

// Stop Conditions
const float maxPitch = 95;
const float minPitch = -95;
const float maxGroundPitch = 85;
const float minGroundPitch = -45;
const float maxGroundRoll = 45;
const float minGroundRoll = -45;
const float minAlpha = -250;
const float maxAlpha = 250;

// Typedefs
#ifdef RealTime
    typedef std::chrono::time_point<std::chrono::system_clock> systemTime;
    typedef std::chrono::duration<double> timeDuration;
#else
    typedef double systemTime;
    typedef double timeDuration;
#endif

/*
// Body Section 1
height = 0.065 (6.5 cm)
length = 0.28  (28 cm)

// Body Section 2
height = 0.065
length = 0.20 (20 cm)

// Total Body
length = 0.48 (48 cm)
neutral point = 0.17 (17 cm)
incidence = 2°
CG in percent MAC = (0.153 - 0.115)/0.13 = 0.29
NP in percent MAC = (0.17  - 0.115)/0.13 = 0.42
static margin = 13%
 
// Wing
b = 0.52 (52 cm)
c = 0.13 (13 cm)
LEposition = 0.115 (11.5 cm)

// Ailerons
b = 0.22 (22 cm)
c = 0.04 (4 cm) 30 %
 
// Horizontal Stabilizer
c = 0.075 (7.5 cm)
b = 0.22  (22 cm)
t = 0.005 (0.5 cm) 6.7 % thickness
LEposition = 0.43 (43 cm)
 
// Elevator
c = 0.02 (2 cm)   25 %
b = 0.22 (22 cm)
 
// Vertical Stabilizer
c1 = 0.11 (11 cm)  (100 %)
c2 = 0.05 (5.0 cm) (45 %)   avegerage to 62.5 %
b = 0.11  (11 cm)
t = 0.005 (0.5 cm) 4.5 % thickness
LEposition = 0.365 (36.5 cm)
 
// rudder
c = 0.03 (3 cm)
b = 0.11 (11 cm)

// Max Reynolds and Mach
Re = 1.225 * 53 * 0.075 / 1.81*10^-5 = 269,000
Mach = 53/343 = 0.15
 
const char *savefile = "output.csv"; // /Users/alexandermclean/Documents/Cpp-Aircraft-Simulation/Aircraft Simulation/Aircraft Simulation
const bool saveOutput = true;

// Aircraft constants
float mass             = 3;
float inertia[4]       = {0.1,0.2,0.3,0.002};

float throttle         = 0.4; // 0 - 1
float maxThrust        = 10;
float actuators[3]     = {-6,0,0}; //{de,da,dr}

// Trim
bool trim = true;
float Vtrim = 25;
float altTrim = 100;
float climbTrim = 0;
float rollTrim = 0;

// World constants
float elevation        = 0;

// Initial conditions
float xInitial         = 0;
float yInitial         = 0;
float zInitial         = -100;
float altInitial       = -zInitial;

float xdotInitial      = 20;
float ydotInitial      = 0;
float zdotInitial      = 0;

float phiInitial       = 0;
float thetaInitial     = 10;
float psiInitial       = 0;

float phidotInitial    = 0;
float thetadotInitial  = 0;
float psidotInitial    = 0;

// World
float posNED[3]        = {xInitial,yInitial,zInitial};
float velNED[3]        = {xdotInitial,ydotInitial,zdotInitial};
float attitude[3]      = {phiInitial,thetaInitial,psiInitial};
float omega[3]         = {phidotInitial,thetadotInitial,psidotInitial};

// Body
float velBody[3];
float pqr[3];

// Truth
float xTruth           = xInitial;
float yTruth           = yInitial;
float zTruth           = zInitial;
float altTruth         = altInitial;

float xdotTruth        = xdotInitial;
float ydotTruth        = ydotInitial;
float zdotTruth        = zdotInitial;

float phiTruth         = phiInitial;
float thetaTruth       = thetaInitial;
float psiTruth         = psiInitial;

float phidotTruth      = phidotInitial;
float thetadotTruth    = thetadotInitial;
float psidotTruth      = psidotInitial;

// Navigation
float xNav             = xInitial;
float yNav             = yInitial;
float zNav             = zInitial;
float altNav           = altInitial;

float xdotNav          = xdotInitial;
float ydotNav          = ydotInitial;
float zdotNav          = zdotInitial;

float phiNav           = phiInitial;
float thetaNav         = thetaInitial;
float psiNav           = psiInitial;

float phidotNav        = phidotInitial;
float thetadotNav      = thetadotInitial;
float psidotNav        = psidotInitial;

// Aero variables
float alpha;
float beta;
float u,v,w;
float p,q,r;

// Forces and Moments
float FaeroWind[3];
float FaeroBody[3];
float MaeroBody[3];

float FgroundBody[3];
float MgroundBody[3];

float FgravityNED[3];
float FgravityBody[3];
float MgravityBody[3];

float sumForce[3];
float sumMoment[3];

// Functions
void updateAeroAngles();
void updateBodyStates();
void updateWorldStates();

// Constants
const float rad2deg = 180/M_PI;
const float deg2rad = M_PI/180;
*/
#endif // InitialConditions_h
