//
//  DynamicsModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/22/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "DynamicsModel.hpp"

void DynamicsModel::dynamics(float* velBody,float* pqr,float* Force,float* Moment,float* Inertia,float m,double dt)
{
    u = velBody[0];
    v = velBody[1];
    w = velBody[2];
    
    p = pqr[0]*deg2rad;
    q = pqr[1]*deg2rad;
    r = pqr[2]*deg2rad;
    
    Fx = Force[0];
    Fy = Force[1];
    Fz = Force[2];

    L = Moment[0];
    M = Moment[1];
    N = Moment[2];
    
    Ix = Inertia[0];
    Iy = Inertia[1];
    Iz = Inertia[2];
    Ixz = Inertia[3];
    
    // in rad/s
    udot = Fx/m + r*v - q*w;
    vdot = Fy/m + p*w - r*u;
    wdot = Fz/m + q*u - p*v;

    den = Ix*Iz-Ixz*Ixz;
    eq1 = L+Ixz*p*q-q*r*(Iz-Iy);
    eq2 = N-Ixz*q*r-p*q*(Iy-Ix);
    
    qdot = (M - p*r*(Ix-Iz) + Ixz*(p*p-r*r))/Iy;
    pdot = Iz/den*eq1 + Ixz/den*eq2;
    rdot = Ixz/den*eq1 + Ix/den*eq2;
    
    // convert to deg/s
    u = u + udot*dt;
    v = v + vdot*dt;
    w = w + wdot*dt;
    p = (p + pdot*dt)*rad2deg;
    q = (q + qdot*dt)*rad2deg;
    r = (r + rdot*dt)*rad2deg;
    
    velBody[0] = u;
    velBody[1] = v;
    velBody[2] = w;
    pqr[0]     = p;
    pqr[1]     = q;
    pqr[2]     = r;
}

void DynamicsModel::integrate(float* pos, float* angle, float* vel, float* omega, double dt){
    x = pos[0];
    y = pos[1];
    z = pos[2];
    
    phi = angle[0];
    theta = angle[1];
    psi = angle[2];
    
    xdot = vel[0];
    ydot = vel[1];
    zdot = vel[2];
    
    phidot   = omega[0];
    thetadot = omega[1];
    psidot   = omega[2];
    
    x = x + xdot*dt;
    y = y + ydot*dt;
    z = z + zdot*dt;
    
    phi   = phi + phidot*dt;
    theta = theta + thetadot*dt;
    psi   = psi + psidot*dt;
    
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    
    angle[0] = phi;
    angle[1] = theta;
    angle[2] = psi;
}

void DynamicsModel::sumForces(float *sum, float *aero, float *ground, float *gravity){
    for(int i=0;i<3;i++)
    {
        sum[i] = aero[i] + ground[i] + gravity[i];
    }
}

float DynamicsModel::getX(void) {
    return x;
}
float DynamicsModel::getY(void) {
    return y;
}
float DynamicsModel::getZ(void) {
    return z;
}
float DynamicsModel::getXdot(void) {
    return xdot;
}
float DynamicsModel::getYdot(void) {
    return ydot;
}
float DynamicsModel::getZdot(void) {
    return zdot;
}

float DynamicsModel::getPhi(void) {
    return phi;
}

float DynamicsModel::getTheta(void) {
    return theta;
}

float DynamicsModel::getPsi(void) {
    return psi;
}

float DynamicsModel::getPhidot(void) {
    return phidot;
}

float DynamicsModel::getThetadot(void) {
    return thetadot;
}

float DynamicsModel::getPsidot(void) {
    return psidot;
}
