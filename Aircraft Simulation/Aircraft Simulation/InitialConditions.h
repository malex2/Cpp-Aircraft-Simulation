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

#endif // InitialConditions_h

// All variables in standard SI units except for angles in degrees

// Aircraft constants
float mass             = 3;
float inertia[4]       = {0.1,0.2,0.3,0.002};

float throttle         = 1.0;
float maxThrust        = 10;
float actuators[3]     = {6,0,0}; //{de,da,dr}

// World constants
float elevation        = 0;

// Update time constants
float runTime          = 3;
float printInterval    = 0.01;
float dynamicsInterval = 0.001;

// Initial conditions
float xInitial         = 0;
float yInitial         = 0;
float zInitial         = -10;
float altInitial       = -zInitial;

float xdotInitial      = 10;
float ydotInitial      = 0;
float zdotInitial      = 0;

float phiInitial       = 0;
float thetaInitial     = 5;
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

// Sim variables
bool continueSim    = true;
int  simLoopCount   = 0;
bool stopSimGround  = false;
