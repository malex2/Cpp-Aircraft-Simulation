//
//  AeroModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/20/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "propulsion_model.hpp"
#include "atmosphere_model.hpp"
#include "initial_conditions.hpp"
#include "time.hpp"
#include "actuator_model.hpp"
#include "ground_model.hpp"

#include "aero_model.hpp"

AeroModelBase::AeroModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pProp   = NULL;
    pAct    = NULL;
    pAtmo   = NULL;
    pTime   = NULL;
    pGnd    = NULL;
    pMap    = pMapInit;
    
    pMap->addLogVar("alpha", &alphaPrint, savePlot, 2);
    pMap->addLogVar("beta", &betaPrint, savePlot, 2);
    pMap->addLogVar("Lift  ", &aeroForce[2], savePlot, 2);
    pMap->addLogVar("Drag  ", &aeroForce[0], savePlot, 2);
    pMap->addLogVar("SideForce", &aeroForce[1], savePlot, 2);
    pMap->addLogVar("Aero Roll Moment", &bodyMoment[0], savePlot, 2);
    pMap->addLogVar("Aero Pitch Moment", &bodyMoment[1], savePlot, 2);
    pMap->addLogVar("Aero Yaw Moment", &bodyMoment[2], savePlot, 2);
    
    util.setUnitClassArray(aeroEuler, zero_init, degrees, 3);
    util.setArray(aeroForce, zero_init, 3);
    alpha.convertUnit(radians);
    beta.convertUnit(radians);
    
    alphaPrint = 0.0;
    betaPrint  = 0.0;
    
    for (int i = 0; i < nCoeff; i ++) { for (int j = 0; j < nDeriv; j++) { aeroMatrix[i][j] = 0.0; } }
    for (int i = 0; i < nCoeff; i ++) { coeffs[i] = 0.0; }
    for (int i = 0; i < nDeriv; i ++)
    {
        inputs[i]  = 0.0;
        Cdarray[i] = 0.0;
        CLarray[i] = 0.0;
        CXarray[i] = 0.0;
        CYarray[i] = 0.0;
        CZarray[i] = 0.0;
        Clarray[i] = 0.0;
        Cmarray[i] = 0.0;
        Cnarray[i] = 0.0;
    }
    inputs[0] = 1.0;
    
    debugFlag = debugFlagIn;
}

void AeroModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pProp   = (PropulsionModel*) pMap->getModel("PropulsionModel");
    pAct    = (ActuatorModel*)   pMap->getModel("ActuatorModel");
    pAtmo   = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pTime   = (Time*)            pMap->getModel("Time");
    pGnd    = (GroundModel*)     pMap->getModel("GroundModel");
    
    updateAeroAngles();
}

bool AeroModelBase::update(void)
{
    updateAeroAngles();
    
    updateAeroCoeffs();
    
    updateAeroForces();
    
    if ( alpha.deg() > maxAlpha || alpha.deg() < minAlpha )
    {
        printf("Aero at %2.2fs: angle of attack out of range: %2.2f\n", pTime->getSimTime(), alpha.deg() );
        return false;
    }
    else { return true; }
}

void AeroModelBase::updateAeroAngles(void)
{
    SpeedType<float>* velBody = pDyn->getVelBodyRelWind();
    
    float V = util.mag(velBody, 3);
    
    if ( velBody[0].val <= zeroTolerance) { alpha.val = 0; }
    else {  alpha.val = atan(velBody[2].val/velBody[0].val); }
    
    if (V <= zeroTolerance) { beta.val = 0; }
    else { beta.val = asin(velBody[1].mps()/V); }
    
    aeroEuler[1].val = alpha.deg();
    aeroEuler[2].val = -beta.deg();
    
    alphaPrint = alpha.deg();
    betaPrint  = beta.deg();
}

void AeroModelBase::updateAeroCoeffs(void)
{
    // place holder
}

void AeroModelBase::updateAeroForces(void)
{
    // Get values from other models
    SpeedType<float>*    velBody = pDyn->getVelBodyRelWind();
    AngleRateType<float>* omega  = pDyn->getBodyRates();
    float V = pDyn->getSpeed().mps();
    float q = pAtmo->getAir()[dynPress];
    float* actuators = pAct->getActuators();
    
    
    // Calculations
    float qS  = q*wingArea;
    float b2v = span/(2*V);
    float c2v = meanChord/(2*V);
    
    // Setup Aero Matrix
    // [ Cd ]   [ Cdo  Cdu Cda Cdb Cdp Cdq Cdr Cdde Cdda Cddr CddT ]   [ 1      ]
    // [ CL ]   [ CLo  CLu CLa CLb CLp CLq CLr CLde CLda CLdr CLdT ]   [ u      ]
    // [ CX ]   [ CXo  CXu CXa CXb CXp CXq CXr CXde CXda CXdr CXdT ]   [ alpha  ]
    // [ CY ] = [ CYo  CYu CYa CYb CYp CYq CYr CYde CYda CYdr CYdT ] x [ beta   ]
    // [ CZ ]   [ CZo  CZu CZa CZb CZp CZq CZr CZde CZda CZdr CZdT ]   [ p      ]
    // [ Cl ]   [ Clo  Clu Cla Clb Clp Clq Clr Clde Clda Cldr CldT ]   [ q      ]
    // [ Cm ]   [ Cmo  Cmu Cma Cmb Cmp Cmq Cmr Cmde Cmda Cmdr CmdT ]   [ r      ]
    // [ Cn ]   [ Cno  Cnu Cna Cnb Cnp Cnq Cnr Cnde Cnda Cndr CndT ]   [ de     ]
    //                                                                 [ da     ]
    //                                                                 [ dr     ]
    //                                                                 [ dT     ]
    
    // enum coeffType {Cd, CL, CX, CY, CZ, Cl, Cm, Cn, nCoeff};
    // enum coeffDervType {constant, du, dalpha, dbeta, dp, dq, dr, delevator, daileron, drudder, dthrottle, nDeriv};
    
    for (int iDeriv = constant; iDeriv != nDeriv; iDeriv++)
    {
        aeroMatrix[iCd][iDeriv] = Cdarray[iDeriv];
        aeroMatrix[iCL][iDeriv] = CLarray[iDeriv];
        aeroMatrix[iCX][iDeriv] = CXarray[iDeriv];
        aeroMatrix[iCY][iDeriv] = CYarray[iDeriv];
        aeroMatrix[iCZ][iDeriv] = CZarray[iDeriv];
        aeroMatrix[iCl][iDeriv] = Clarray[iDeriv];
        aeroMatrix[iCm][iDeriv] = Cmarray[iDeriv];
        aeroMatrix[iCn][iDeriv] = Cnarray[iDeriv];
    }
    
    if ( V >= zeroTolerance)
    {
        // Setup inputs vector
        // uhat
        inputs[1] = velBody[1].mps()/V;
    
        // alpha, beta
        inputs[2] = alpha.rad();
        inputs[3] = beta.rad();
    
        // phat, qhat, rhat
        inputs[4] = omega[0].rps()*b2v; // p*b/(2*V)
        inputs[5] = omega[1].rps()*c2v; // q*c/(2*V)
        inputs[6] = omega[2].rps()*b2v; // r*b/(2*V)
    
        // de, da, dr, dT
        inputs[7] = actuators[de]*util.deg2rad;
        inputs[8] = actuators[da]*util.deg2rad;
        inputs[9] = actuators[2]*util.deg2rad;
        inputs[10] = pProp->getThrottle();
    
        // Compute coefficients
        util.mmult(coeffs, *aeroMatrix, inputs, 8, 11);
    
        // Lift and drag coefficients to body frame
        float liftDragWind[3] = {coeffs[0], 0, coeffs[1]};
        float liftDragBody[3];
        pRotate->windToBody(liftDragBody, liftDragWind);
    
        // CX, CY, CZ
        bodyForceCoeff[0] = coeffs[2] + liftDragBody[0];
        bodyForceCoeff[1] = coeffs[3] + liftDragBody[1];
        bodyForceCoeff[2] = coeffs[4] + liftDragBody[2];
    
        // Cl, Cm, Cn
        bodyMomentCoeff[0] = coeffs[5];
        bodyMomentCoeff[1] = coeffs[6];
        bodyMomentCoeff[2] = coeffs[7];
    
        // Forces and Moments
        bodyForce[0] = -bodyForceCoeff[0]*qS;
        bodyForce[1] = -bodyForceCoeff[1]*qS;
        bodyForce[2] = -bodyForceCoeff[2]*qS;

        bodyMoment[0] = bodyMomentCoeff[0] * qS * span;
        bodyMoment[1] = bodyMomentCoeff[1] * qS * meanChord;
        bodyMoment[2] = bodyMomentCoeff[2] * qS * span;
    
        // Aero Forces
        pRotate->bodyToWind(aeroForce, bodyForce);
        util.vgain(aeroForce, -1.0f, 3);
    }
}

SimpleRCAeroModel::SimpleRCAeroModel(ModelMap *pMapInit, bool debugFlagIn) : AeroModelBase(pMapInit, debugFlagIn)
{
    dCd_dCL  = 0; // computed dynamically
    Cdarray[constant]  = 0.04;       // Cdo
    Cdarray[dalpha]    = 0.0;        // Cda  computed dynamically
    Cdarray[dq]        = 0.0;        // Cdq  computed dynamically
    Cdarray[delevator] = 0.0;        // Cdde computed dynamically
    
    CLarray[constant]  = 0.31;       // CLo
    CLarray[dalpha]    = 4.2072;     // CLa
    CLarray[dq]        = 6.7408;     // CLq
    CLarray[delevator] = 0.50583;    // CLde
    
    Cmarray[constant]  = 0.02221337; // Cmo
    Cmarray[dalpha]    = -0.82203;   // Cma
    Cmarray[dq]        = -8.9323;    // Cmq
    Cmarray[delevator] = -1.1669;    // Cmde
    
    CYarray[dbeta]     = -0.35968;    // CYb
    CYarray[dp]        = -0.00059002; // CYp
    CYarray[dr]        = 0.40526;     // CYr
    CYarray[daileron]  = -0.13034;    // CYda
    CYarray[drudder]   = 0.28305;     // CYdr
    
    Clarray[dbeta]     = -0.04194;    // Clb
    Clarray[dp]        = -0.39218;    // Clp
    Clarray[dr]        = 0.15978;     // Clr
    Clarray[daileron]  = 0.40029;     // Clda
    Clarray[drudder]   = 0.0033462;   // Cldr
    
    Cnarray[dbeta]    = 0.18222;      // Cnb
    Cnarray[dp]       = -0.063944;    // Cnp
    Cnarray[dr]       = -0.20752;     // Cnr
    Cnarray[daileron] = 0.065812;     // Cnda
    Cnarray[drudder]  = -0.15261;     // Cndr
}

void SimpleRCAeroModel::updateAeroCoeffs(void)
{
    // Compute CL
    float CL;
    float q = pDyn->getBodyRates()[1].rps();
    float V = pDyn->getSpeed().mps();
    
    CL = CLarray[constant] +
    CLarray[dalpha]*alpha.rad() +
    CLarray[dq]*q*meanChord/(2*V) +
    CLarray[delevator]*actuators_init[0]*util.deg2rad;
    
    // Copmute derivative of Cd w.r.t. CL
    dCd_dCL = 2*CL/(M_PI*AR);
    
    // Update drag derivatives
    Cdarray[dalpha]    = CLarray[dalpha]    * dCd_dCL;
    Cdarray[dq]        = CLarray[dq]        * dCd_dCL;
    Cdarray[delevator] = CLarray[delevator] * dCd_dCL;
}

RCAeroModel::RCAeroModel(ModelMap *pMapInit, bool debugFlagIn) : AeroModelBase(pMapInit, debugFlagIn)
{
    // Transpose tables
    util.mtran(*AeroTable_Longitudinal, *AeroTable_LongitudinalT, 10, 8);
    util.mtran(*AeroTable_Lateral, *AeroTable_LateralT, 9, 8);
    util.mtran(*AeroTable_Control, *AeroTable_ControlT, 7, 8);
    util.mtran(*elevatorTable, *elevatorTableT, 2, 8);
    util.mtran(*aileronTable, *aileronTableT, 3, 7);
    util.mtran(*rudderTable, *rudderTableT, 3, 7);
}

void RCAeroModel::updateAeroCoeffs(void)
{
    float Re = pAtmo->getRe();
    float elevator = actuators_init[0];
    float aileron = actuators_init[1];
    float rudder = actuators_init[2];
    
    // Drag coefficient lookup
    Cdarray[constant] = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCdo][0], Re, nReynolds);
    
    // Lift coefficient lookup
    CLarray[constant] = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLo][0], Re, nReynolds);
    CLarray[dalpha]   = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLa][0], Re, nReynolds);
    CLarray[dq]       = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLq][0], Re, nReynolds);
    
    // X Force coefficient lookup
    CXarray[du]        = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCXu][0], Re, nReynolds);
    CXarray[dalpha]    = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCXa][0], Re, nReynolds);
    CXarray[delevator] = util.interpolate(deVec, &elevatorTable[lookupCXde][0], elevator, nDe);
    CXarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCXda][0] , aileron , nDa);
    CXarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCXdr][0]  , rudder  , nDr);
    
    // Y (Side) Force coefficient lookup
    CYarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYb][0], Re, nReynolds);
    CYarray[dp]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYp][0], Re, nReynolds);
    CYarray[dr]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYr][0], Re, nReynolds);
    CYarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCYda][0], Re, nReynolds);
    CYarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCYdr][0], Re, nReynolds);
    
    // Z Force coefficient lookup
    CZarray[du]        = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCZu][0], Re, nReynolds);
    CZarray[delevator] = util.interpolate(deVec, &elevatorTable[lookupCZde][0], elevator, nDe);
    CZarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCZda][0] , aileron , nDa);
    CZarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCZdr][0]  , rudder  , nDr);
    
    // Roll Moment coefficient lookup
    Clarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClb][0], Re, nReynolds);
    Clarray[dp]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClp][0], Re, nReynolds);
    Clarray[dr]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClr][0], Re, nReynolds);
    Clarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupClda][0], Re, nReynolds);
    Clarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCldr][0], Re, nReynolds);
    
    // Pitch Moment coefficient lookup
    Cmarray[constant]  = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCmo][0], Re, nReynolds);
    Cmarray[dalpha]    = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCma][0], Re, nReynolds);
    Cmarray[dq]        = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCmq][0], Re, nReynolds);
    Cmarray[delevator] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCmde][0], Re, nReynolds);
    Cmarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCmda][0] , aileron , nDa);
    Cmarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCmdr][0]  , rudder  , nDr);
    
    // Rudder Moment coefficient lookup
    Cnarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnb][0], Re, nReynolds);
    Cnarray[dp]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnp][0], Re, nReynolds);
    Cnarray[dr]       = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnr][0], Re, nReynolds);
    Cnarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCnda][0], Re, nReynolds);
    Cnarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCndr][0], Re, nReynolds);
}

/*
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
*/
