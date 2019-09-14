//
//  AeroModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/20/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "AeroModel.hpp"

void AeroModel::rcPlane(float *pqr,float *controls, float *FaeroWind, float *MaeroBody){
    derad = controls[0]*deg2rad;
    darad = controls[1]*deg2rad;
    drrad = controls[2]*deg2rad;
    
    p = pqr[0];
    q = pqr[1];
    r = pqr[2];
    
    alpharad = alpha*deg2rad;
    betarad = beta*deg2rad;
    
    phat = b*p*deg2rad/(2*V);
    rhat = b*r*deg2rad/(2*V);
    qhat = cbar*q*deg2rad/(2*V);
    
    Cdo = 0.02;
    
    CLo = 0.2206;
    CLa = 5.06;
    CLde = 0.2865;
    CLq = 3.98;
    
    Cmo = 0.14;
    Cma = -1.15;
    Cmde = -0.66;
    Cmq = -10.14;
    
    CYb = -0.4;
    CYp = 0;
    CYr = 0;
    CYda = 0;
    CYdr = 0.2;
    
    Clb = -0.1;
    Clp = -0.5;
    Clr = 0.4;
    Clda = 0.51;
    Cldr = 0.04;

    Cnb = 0.103;
    Cnp = -0.3;
    Cnr = -0.2;
    Cnda = 0;
    Cndr = -0.0018;

    
    Cd = Cdo + CL*CL/(M_PI*AR);
    CL = CLo + CLa*alpharad + CLq*qhat + CLde*derad;
    Cm = Cmo + Cma*alpharad + Cmq*qhat + Cmde*derad;
    CY = CYb*betarad + CYr*rhat + CYp*phat + CYda*darad + CYdr*drrad;
    Cl = Clb*betarad + Clr*rhat + Clp*phat + Clda*darad + Cldr*drrad;
    Cn = Cnb*betarad + Cnr*rhat + Cnp*phat + Cnda*darad + Cndr*drrad;
    
    qS = 0.5*rho*V*V*S;
    
    Lift = qS*CL;
    Drag = qS*Cd;
    SideForce = qS*CY;
    
    L = qS*b*Cl;
    M = qS*cbar*Cm;
    N = qS*b*Cn;
    
    FaeroWind[0] = -Drag;
    FaeroWind[1] = -SideForce;
    FaeroWind[2] = -Lift;
    
    MaeroBody[0] = L;
    MaeroBody[1] = M;
    MaeroBody[2] = N;
}

void AeroModel::Dummy(double curTime, const int size,float *dummyTimes, float *dummyForceMoments, float *FaeroWind, float *MaeroBody){
    // Get forces    
    for(int i=0;i<6;i++)
    {
            for(int j=0;j<size;j++)
            {
               if(*(dummyTimes+i*size+j)<=curTime)
               {
                   if (i<3) FaeroWind[i]=*(dummyForceMoments+i*size+j);
                   else     MaeroBody[i-3]=*(dummyForceMoments+i*size+j);
               }
            }
    }
}

void AeroModel::Angles(float* velBody){
    u = velBody[0];
    v = velBody[1];
    w = velBody[2];
    V = sqrt(u*u + v*v + w*w);
    if (u<=zeroTolerance) alpha = 0;
    else alpha = atan(w/u)*rad2deg;
    
    if (v<=zeroTolerance) beta = 0;
    else beta = asin(v/V)*rad2deg;
    
}

float AeroModel::getAlpha(){
    return alpha;
}

float AeroModel::getBeta(){
    return beta;    
}

