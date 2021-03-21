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
const double runTime_init          = 120;
const double printInterval_init    = 0.1;
const double saveInterval_init     = 0.01;
const double dynamicsInterval_init = 0.001;
const double clock_dt              = 0.001; // time interval when not in real time mode

// Print options
const bool saveOutput  = true;
const bool printOutput = true;
const bool plotOutput  = false;
const std::string savefile = "output.csv";

// Initial States
const double velNED_init[3]  = {0, 0, 0};
const double posBody_init[3] = {28.5997222, -81.3394444, 1.968}; // deg, deg, ft
const double eulerAngles_init[3] = {0, 0.0, 0}; // {0, 6.507, 0};
const double eulerRates_init[3]  = {0, 0, 0};
const double actuators_init[4]   = {0.0, 0.0, 0.0, 0.0}; //{de,da,dr,dT}
const int inputMode_init = 2; // 0 - external, 1 - keyboard, 2 - table

// Trim
// Uses initial position above
// Uses initial heading from above
const bool trim = false;

static SpeedType<double> trimSpeed(0.0,metersPerSecond); // Ground speed
static AngleType<double> trimRoll(0, degrees);         // Roll/bank angle
static AngleType<double> trimGamma(0, degrees);        // Climb angle

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
    {0.0000907 , 0.0      , 0.0        },
    {0.0       , 0.0005005, 0.0        },
    {0.0       , 0.0      , 0.0005422  }};

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
const double elevation_init = 0;
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
double mass             = 3;
double inertia[4]       = {0.1,0.2,0.3,0.002};

double throttle         = 0.4; // 0 - 1
double maxThrust        = 10;
double actuators[3]     = {-6,0,0}; //{de,da,dr}

// Trim
bool trim = true;
double Vtrim = 25;
double altTrim = 100;
double climbTrim = 0;
double rollTrim = 0;

// World constants
double elevation        = 0;

// Initial conditions
double xInitial         = 0;
double yInitial         = 0;
double zInitial         = -100;
double altInitial       = -zInitial;

double xdotInitial      = 20;
double ydotInitial      = 0;
double zdotInitial      = 0;

double phiInitial       = 0;
double thetaInitial     = 10;
double psiInitial       = 0;

double phidotInitial    = 0;
double thetadotInitial  = 0;
double psidotInitial    = 0;

// World
double posNED[3]        = {xInitial,yInitial,zInitial};
double velNED[3]        = {xdotInitial,ydotInitial,zdotInitial};
double attitude[3]      = {phiInitial,thetaInitial,psiInitial};
double omega[3]         = {phidotInitial,thetadotInitial,psidotInitial};

// Body
double velBody[3];
double pqr[3];

// Truth
double xTruth           = xInitial;
double yTruth           = yInitial;
double zTruth           = zInitial;
double altTruth         = altInitial;

double xdotTruth        = xdotInitial;
double ydotTruth        = ydotInitial;
double zdotTruth        = zdotInitial;

double phiTruth         = phiInitial;
double thetaTruth       = thetaInitial;
double psiTruth         = psiInitial;

double phidotTruth      = phidotInitial;
double thetadotTruth    = thetadotInitial;
double psidotTruth      = psidotInitial;

// Navigation
double xNav             = xInitial;
double yNav             = yInitial;
double zNav             = zInitial;
double altNav           = altInitial;

double xdotNav          = xdotInitial;
double ydotNav          = ydotInitial;
double zdotNav          = zdotInitial;

double phiNav           = phiInitial;
double thetaNav         = thetaInitial;
double psiNav           = psiInitial;

double phidotNav        = phidotInitial;
double thetadotNav      = thetadotInitial;
double psidotNav        = psidotInitial;

// Aero variables
double alpha;
double beta;
double u,v,w;
double p,q,r;

// Forces and Moments
double FaeroWind[3];
double FaeroBody[3];
double MaeroBody[3];

double FgroundBody[3];
double MgroundBody[3];

double FgravityNED[3];
double FgravityBody[3];
double MgravityBody[3];

double sumForce[3];
double sumMoment[3];

// Functions
void updateAeroAngles();
void updateBodyStates();
void updateWorldStates();

// Constants
const double rad2deg = 180/M_PI;
const double deg2rad = M_PI/180;
*/
#endif // InitialConditions_h
