//
//  GroundModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/4/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "GroundModel.hpp"
#include "RotateFrame.hpp"

bool GroundModel::flatGround(float alt,float m, float* angles, float* omega, float* velBody, float* FaeroBody, float *groundForceBody, float *groundMoment){
    // Note: alt is altitude of aircraft relative to the ground. i.e. when alt = 0, aircraft is on ground
    
    RotateFrame rotate;
    
    phi = angles[0];
    theta = angles[1];
    phidot = omega[0];
    thetadot = omega[1];
    
    rotate.RotateBodyToInertial(phi*deg2rad,theta*deg2rad,0,velBody,velGround);
    Vx = velGround[0];
    Vy = velGround[1];
    Vz = velGround[2];
    
    dthetarad = (theta - thetag)*deg2rad;
    b1R = 2*zeta1R*sqrt(k1R*m);
    b1L = 2*zeta1L*sqrt(k1L*m);
    b2  = 2*zeta2*sqrt(k2*m);
    
     Vz1R = Vz - thetadot*deg2rad*L1 + phidot*deg2rad*(LR+Lg1*sin(phi*deg2rad));
     Vz1L = Vz - thetadot*deg2rad*L1 - phidot*deg2rad*(LL-Lg1*sin(phi*deg2rad));
     Vz2  = Vz + thetadot*deg2rad*L2 - phidot*deg2rad*Lg2*sin(phi*deg2rad);
    
    
    altTrue = alt + cgz;
    altL1L = altTrue - Lg1 - L1*sin(dthetarad)  + L1*sin(thetag*deg2rad) + Lg1*(1-cos(phi*deg2rad)) + LL*sin(phi*deg2rad);
    altL1R = altTrue - Lg1 - L1*sin(dthetarad)  + L1*sin(thetag*deg2rad) - Lg1*(1-cos(phi*deg2rad)) - LL*sin(phi*deg2rad);
    altL2 = altTrue - Lg2 - L2*sin(dthetarad) - L2*sin(thetag*deg2rad) + Lg2*(1-cos(phi*deg2rad));
    
    if (altL1L <=0) { N1L = k1L*altL1L + b1L*(-Vz1L); }
    else { N1L = 0; }
    
    if (altL1R <=0) { N1R = k1R*altL1R + b1R*(-Vz1R); }
    else { N1R = 0; }
    
    if (altL2 <=0) { N2 = k1R*altL2 + b2*(-Vz2); }
    else { N2 = 0; }

    Fx1L = N1L*mux;
    Fx1R = N1R*mux;
    Fx2  = N2*mux;
    
    Fy1L = N1L*muy;
    Fy1R = N1R*muy;
    Fy2  = N2*muy;
    
    sumFrictionX = (N1L + N1R + N2)*mux; // Inherently a negative value
    sumFrictionY = (N1L + N1R + N2)*muy; // Inherently a negative value
    
    rotate.RotateBodyToInertial(phi*deg2rad,theta*deg2rad,0,FaeroBody,FaeroGround);
    xForce = FaeroGround[0];
    yForce = FaeroGround[1];
    
   signx = Vx/sqrt(Vx*Vx);
   signy = Vy/sqrt(Vy*Vy);
    
    // If Vx is positive, friction stays negative
    if(abs(Vx) > zeroTolerance) {
        Fx1L *= signx;
        Fx1R *= signx;
        Fx2 *= signx;
    }
    // If Vy is positive, friction stays negative
    if(abs(Vy) > zeroTolerance) {
        Fy1L *= signy;
        Fy1R *= signy;
        Fy2 *= signy;
    }
    
    sumFrictionX = Fx1L + Fx1R + Fx2;
    sumFrictionY = Fy1L + Fy1R + Fy2;
    
     if (abs(Vx) <= zeroTolerance && xForce <= -sumFrictionX) { sumFrictionX = -xForce; }
     if (abs(Vy) <= zeroTolerance && yForce <= -sumFrictionY) { sumFrictionY = -yForce; }

    groundForce[0] = sumFrictionX;
    groundForce[1] = sumFrictionY;
    groundForce[2] = N1L+N1R+N2;
    
    rotate.RotateInertialToBody(phi*deg2rad,theta*deg2rad,0,groundForceBody,groundForce);
    
    M = (-(N1R+N1L)*L1 + N2*L2)*cos(dthetarad) - Fx1L*Lg1 - Fx1R*Lg1 - Fx2*L2;
    L = -N1L*L1 + N1R*L1 - (Fy1L*Lg1 + Fy1R*Lg1 + Fy2*Lg2);
    
    //    std::cout<<altL1L<<" "<<altL1R<<"\t\t";
      //  std::cout<<N1L<<" "<<N1R<<"\t\t";
    
   // std::cout<<Vy<<"\t\t"<<Fy2<<"\t\t"<<L<<"\t\t"<<phi<<"\n";
 //  std::cout<<theta<<" "<<phi<<"\n";
    groundMoment[0] = L;
    groundMoment[1] = M;
    groundMoment[2] = 0;
    
    // Determine if aircraft is in unsaveable orientation and stop simulation
    if (altL1L <=0 || altL1R <=0 || altL2 <=0) { onGround = true; }
    else { onGround = false; }
    
    if (onGround){
        if (abs(phi) > 45 || abs(theta) > 45){ stopSim = true; }
    }
    
    return stopSim;
    
    
    
    
    
    
    
}
