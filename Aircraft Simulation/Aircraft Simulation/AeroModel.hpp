//
//  AeroModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/20/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef AeroModel_hpp
#define AeroModel_hpp

#include <stdio.h>
#include <math.h>
#include <iostream>

#endif /* AeroModel_hpp */

class AeroModel
{
public:
    void Dummy(double curTime, const int size,float *dummyTimes, float *dummyForceMoments, float *FaeroW, float *MaeroW);
    void rcPlane(float *pqr,float *controls,float *FaeroWind, float *MaeroBody);
    
    void Angles(float* velBody);
    float getAlpha();
    float getBeta();
private:
    const float zeroTolerance = 1e-6;
    const float rad2deg = 180/M_PI;
    const float deg2rad = M_PI/180;
    float p,q,r;
    float u,v,w,V;
    float phat,qhat,rhat;
    float alpha, beta;
    float alpharad,betarad;
    
    float de,da,dr,derad,darad,drrad;
    float qS;
    float CLo, Cdo, CLa, CLde, CLq, Cmo, Cma, Cmde, Cmq;
    float CYb, CYp, CYr, CYda, CYdr, Clb, Clp, Clr, Clda, Cldr, Cnb, Cnp, Cnr, Cnda, Cndr;
    
    float CL,Cm,Cd,CY,Cn,Cl;
    float Lift,Drag,SideForce,L,M,N;
    
    float rho = 1.225;
    float S = 0.0675;
    float b = 0.52;
    float cbar = S/b;
    float AR = b*b/S;
};
