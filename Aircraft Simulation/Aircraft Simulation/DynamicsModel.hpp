//
//  DynamicsModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/22/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef DynamicsModel_hpp
#define DynamicsModel_hpp

#include <stdio.h>
#include <iostream>
#include <math.h>

#endif /* DynamicsModel_hpp */
class DynamicsModel
{
public:
    void dynamics(float* velBody,float* pqr,float* Force,float* Moment,float* Inertia,float m, double dt);
    void integrate(float* pos, float* angle, float* vel, float* omega, double dt);
    void sumForces(float* sum,float* aero,float* ground,float* gravity);
    
    float getX(void);
    float getY(void);
    float getZ(void);
    float getXdot(void);
    float getYdot(void);
    float getZdot(void);
    
    float getPhi();
    float getTheta();
    float getPsi();
    float getPhidot();
    float getThetadot();
    float getPsidot();
    
private:
    const float deg2rad = M_PI/180;
    const float rad2deg = 180/M_PI;
    float u,v,w;
    float udot,vdot,wdot;
    float p,q,r;
    float pdot,qdot,rdot;
    float Fx,Fy,Fz;
    float Ix,Iy,Iz,Ixz;
    float L,M,N;
    float den,eq1,eq2;
 
    float x, y, z;
    float xdot, ydot, zdot;
    float phi,theta,psi;
    float phidot,thetadot,psidot;
};
