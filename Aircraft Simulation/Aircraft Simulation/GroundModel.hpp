//
//  GroundModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/4/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef GroundModel_hpp
#define GroundModel_hpp

#include <stdio.h>
#include <math.h>
#include <iostream>

#endif /* GroundModel_hpp */

class GroundModel
{
public:
    bool flatGround(float alt, float m, float* angle,float* omega, float* velBody, float* FaeroBody, float* groundForce, float* groundMoment);
private:
    const float rad2deg = 180/M_PI;
    const float deg2rad = M_PI/180;
    const float zeroTolerance = 1e-6;
    float groundForce[3];
    float FaeroGround[3];
    float velGround[3];
    float xForce;
    float yForce;
    float sumFrictionX;
    float sumFrictionY;
    float Vx;
    float Vy;
    float Vz;
    float theta;
    float phi;
    float thetadot;
    float phidot;
    
    float Fx1L;
    float Fx1R;
    float Fx2;
    float Fy1L;
    float Fy1R;
    float Fy2;
    float signx;
    float signy;
    
   const float cgz = 0.09; // Height of center of gravity above ground when aircraft is on ground
   const float Lg1 = 0.1; // Length of front landing gear
   const float Lg2 = 0.06; // Length of rear landing gear
   const float L1 = 0.06; // Distance of front landing gear from CG
   const float L2 = 0.29; // Distance of rear landing gear from CG
    const float LL = 0.05; // Distance of left landing gear from CG
    const float LR = 0.05; // Distance of right landing gear from CG
   const float thetag = asin((Lg1-Lg2)/(L1+L2))*rad2deg; // Pitch of aircraft on flat ground
    
    const float k1R = 10000;
    const float k1L = 10000;
    const float k2  = 10000;
    const float zeta1R = 1;
    const float zeta1L = 1;
    const float zeta2  = 1;
    
    float dthetarad;
    
    float mux = 0.08; // 0.02-0.08
    float muy = 1.08; // 0.02-0.08
    float b1R;
    float b1L;
    float b2;
    
    float altTrue; // Altitude of CG above ground
    float altLG1; // Altitude of front landing gear
    float altLG2; // Altitude of rear landing gear

    float altL1L; // Altitude of front left landing gear
    float altL1R; // Altitude of front right landing gear
    float altL2; // Altitude of rear landing gear
    
    float Vz1R;
    float Vz1L;
    float Vz2;
    
    float N1L; // Normal force on front left landing gear
    float N1R; // Normal force on right left landing gear
    float N2; // Normal force on rear landing gear
    float Fk1L; // Frictional force on front left landing gear
    float Fk1R; // Frictional force on right left landing gear
    float Fk2; // Frictional force on rear landing gear
    float M; // Pitch moment of ground on aircraft
    float L; // Roll moment of ground on aircraft
    
    bool onGround;
    bool stopSim = false;
    
};
