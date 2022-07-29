//
//  performance.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/31/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "performance.hpp"
#include "aero_model.hpp"
#include "propulsion_model.hpp"
#include "atmosphere_model.hpp"
#include "actuator_model.hpp"
#include "rotate_frame.hpp"
#include "time.hpp"
#include "dynamics_model.hpp"

Performance::Performance(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pAero   = NULL;
    pProp   = NULL;
    pAtmo   = NULL;
    pAct    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    
    pMap     = pMapInit;
    
    //trimV = trimSpeed;
    //pMap->addLogVar("xForce", &bodyForce[0], printSavePlot, 3);
    
    debugFlag = debugFlagIn;
}

void Performance::initialize(void)
{
    pDyn    = (DynamicsModel*)       pMap->getModel("DynamicsModel");
    pAero   = (AeroModelBase*)       pMap->getModel("AeroModel");
    pProp   = (PropulsionModelBase*) pMap->getModel("PropulsionModel");
    pAtmo   = (AtmosphereModel*)     pMap->getModel("AtmosphereModel");
    pAct    = (ActuatorModelBase*)   pMap->getModel("ActuatorModel");
    pRotate = (RotateFrame*)         pMap->getModel("RotateFrame");
    pTime   = (Time*)                pMap->getModel("Time");
    
    util.setArray(controls, actuators_init, nActuators);
    
    updateConstants();
}

bool Performance::update(void)
{
    return true;
}

void Performance::updateConstants(void)
{
    qdyn = 0.5 * pAtmo->getAir()[AtmosphereModel::density] * trimV.mps() * trimV.mps();
    
    qS = qdyn*wingArea;
    
    Cw = pDyn->getMass()*pAtmo->getGravity() / qS;
    
    //CTmax = pProp->getMaxThrust() / qS;
}

void Performance::LDmax(void)
{
    // Cdo = CL^2/pi*e*AR
    // sqrt(Cdo*PI*e*AR) = CL = W/(qS)
    // qS = W / sqrt(Cdo*PI*e*AR)
    // q = 0.5*rho*V^2 = W / (S * sqrt(Cdo*PI*e*AR) )
    // V = sqrt( 2*W/(rho*S*sqrt(Cdo*PI*e*AR)) )
    updateAeroCoeff();
    double rho = pAtmo->getAir()[AtmosphereModel::density];
    double W = pDyn->getMass()*pAtmo->getGravity();
    double velLDmax = sqrt( 2*W/(rho*wingArea*sqrt(Cdo*M_PI*e*AR)) );
 
    util.print(&velLDmax, 1, "L/D max Velocity:");
}

void Performance::trim(SpeedType<double> trimV)
{
    updateConstants();
    
    updateAeroCoeff();
    
    // Longitudinal Solution
    // [ CW - CLo ] = [ CLa   CLde  ] x [   alphaTrim ]
    // [ -Cmo    ]   [ Cma   Cmde  ]    [   deTrim    ]
    
    lonConsts[0] = Cw - CLo;
    lonConsts[1] = -Cmo;
    
    lonMatrix[0][0] = CLa;   lonMatrix[0][1] = CLde;
    lonMatrix[1][0] = Cma;   lonMatrix[1][1] = Cmde;
    
    util.LUdecomp(lonTrimSoln, *lonMatrix, lonConsts, 2);
    
    // Update longitudinal solution
    trimAero[1].val = lonTrimSoln[0]/util.deg2rad; // angle of attack
    controls[RCActuatorModel::de]    = lonTrimSoln[1]/util.deg2rad; // elevator
    
    // Update coefficient values and forces
    updateForces();
    
    // Throttle
    controls[RCActuatorModel::dT] = CT / CTmax;
    
    // Update Attitude
    trimAttitude[1].val = trimAero[1].deg() + trimGamma.deg();  // Pitch
    trimAttitude[2].val = eulerAngles_init[2];                  // Yaw
    
    util.print(forces, 4, "Thrust Drag Lift Weight");
    util.print(&trimSpeed, 1, "Trim Speed");
    util.print(trimAero, 3, "Aero Trim (phi alpha beta)");
    util.print(trimAttitude, 3, "Attitude Trim (roll pitch yaw)");
    util.print(controls, 4, "Actuator Trim (de da dr dT)");
    
    // Set trim conditions
    //setTrim();
}

void Performance::updateAeroCoeff(void)
{
    pAero->update();
    aeroMatrix = pAero->getAeroMatrix();
    
    Cdo = *(aeroMatrix + iCd*nDeriv + constant);
    
    CLo  = *(aeroMatrix + iCL*nDeriv + constant);
    CLa  = *(aeroMatrix + iCL*nDeriv + dalpha);
    CLq  = *(aeroMatrix + iCL*nDeriv + dbq);
    CLde = *(aeroMatrix + iCL*nDeriv + delevator);
    
    Cmo  = *(aeroMatrix + iCm*nDeriv + constant);
    Cma  = *(aeroMatrix + iCm*nDeriv + dalpha);
    Cmq  = *(aeroMatrix + iCm*nDeriv + dbq);
    Cmde = *(aeroMatrix + iCm*nDeriv + delevator);
    
    CYb   = *(aeroMatrix + iCY*nDeriv + dbeta);
    CYp   = *(aeroMatrix + iCY*nDeriv + dbp);
    CYr   = *(aeroMatrix + iCY*nDeriv + dbr);
    CYda  = *(aeroMatrix + iCY*nDeriv + daileron);
    CYdr  = *(aeroMatrix + iCY*nDeriv + drudder);
    
    Clb   = *(aeroMatrix + iCl*nDeriv + dbeta);
    Clp   = *(aeroMatrix + iCl*nDeriv + dbp);
    Clr   = *(aeroMatrix + iCl*nDeriv + dbr);
    Clda  = *(aeroMatrix + iCl*nDeriv + daileron);
    Cldr  = *(aeroMatrix + iCl*nDeriv + drudder);
    
    Cnb   = *(aeroMatrix + iCn*nDeriv + dbeta);
    Cnp   = *(aeroMatrix + iCn*nDeriv + dbp);
    Cnr   = *(aeroMatrix + iCn*nDeriv + dbr);
    Cnda  = *(aeroMatrix + iCn*nDeriv + daileron);
    Cndr  = *(aeroMatrix + iCn*nDeriv + drudder);
}

void Performance::updateForces(void)
{
    CL = CLo + CLa*trimAero[1].rad() + CLq*qhat + CLde*controls[RCActuatorModel::de]*util.deg2rad;
    Cd = Cdo + CL*CL/(M_PI*e*AR);
    CT = Cd;
    
    LDratio = CL/Cd;
    
    forces[pThrust] = qS*CT;
    forces[pDrag]   = qS*Cd;
    forces[pLift]   = qS*CL;
    forces[pWeight] = qS*Cw;
}

void Performance::trimModels(void)
{
    //pDyn -> setEulerAngles(trimAttitude);
    //pDyn -> setSpeed(trimV);
    pAero-> setAeroEuler(trimAero);
    pAct -> setCommands(controls);
}
