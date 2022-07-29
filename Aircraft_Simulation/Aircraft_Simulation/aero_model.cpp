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

// **********************************************************************
// Aerodynamic Model Base
// **********************************************************************
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
    
    //pMap->addLogVar("alpha", &alphaPrint, savePlot, 2);
    //pMap->addLogVar("beta", &betaPrint, savePlot, 2);
    //pMap->addLogVar("Lift  ", &aeroForce[2], savePlot, 2);
    //pMap->addLogVar("Drag  ", &aeroForce[0], savePlot, 2);
    //pMap->addLogVar("SideForce", &aeroForce[1], savePlot, 2);
    //pMap->addLogVar("Aero Roll Moment", &bodyMoment[0], savePlot, 2);
    //pMap->addLogVar("Aero Pitch Moment", &bodyMoment[1], savePlot, 2);
    //pMap->addLogVar("Aero Yaw Moment", &bodyMoment[2], savePlot, 2);
    
    util.setUnitClassArray(aeroEuler, zero_init, degrees, 3);
    util.setArray(aeroForce, zero_init, 3);
    alpha.convertUnit(radians);
    beta.convertUnit(radians);
    
    alphaPrint = 0.0;
    betaPrint  = 0.0;
    refArea    = 0.0;
    
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
    pDyn    = (DynamicsModel*)       pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)         pMap->getModel("RotateFrame");
    pProp   = (PropulsionModelBase*) pMap->getModel("PropulsionModel");
    pAct    = (ActuatorModelBase*)   pMap->getModel("ActuatorModel");
    pAtmo   = (AtmosphereModel*)     pMap->getModel("AtmosphereModel");
    pTime   = (Time*)                pMap->getModel("Time");
    pGnd    = (GroundModel*)         pMap->getModel("GroundModel");
    
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
    double velBodyRelWind[3];
    
    util.vSubtract(velBodyRelWind, pDyn->getVelBody(), pAtmo->getVelWindBody(), 3);
    
    double V = util.mag(velBodyRelWind, 3);

    // Anlge of Attack
    alpha.val = atan2(velBodyRelWind[2], velBodyRelWind[0]);
    
    // Side Slip
    if (V <= zeroTolerance) { beta.val = 0; }
    else { beta.val = asin(velBodyRelWind[1]/V); }
    
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
    double* omega  = pDyn->getBodyRates();
    double V = pDyn->getSpeed();
    double q = pAtmo->getAir()[AtmosphereModel::dynPress];
    double* actuators = pAct->getActuators();
    
    // Calculations
    double qS  = q*refArea;
    double b2v = span/(2*V);
    double c2v = meanChord/(2*V);
    
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
        inputs[constant] = 1;
        
        // uhat
        inputs[du] = V/Vref;
    
        // alpha, beta
        inputs[dalpha] = alpha.rad();
        inputs[dbeta] = beta.rad();
    
        // phat, qhat, rhat
        inputs[dbp] = omega[0]*b2v; // p*b/(2*V)
        inputs[dbq] = omega[1]*c2v; // q*c/(2*V)
        inputs[dbr] = omega[2]*b2v; // r*b/(2*V)
    
        // de, da, dr, dT
        inputs[delevator] = actuators[RCActuatorModel::de]*util.deg2rad;
        inputs[daileron]  = actuators[RCActuatorModel::da]*util.deg2rad;
        inputs[drudder]   = actuators[RCActuatorModel::dr]*util.deg2rad;
        inputs[dthrottle] = actuators[RCActuatorModel::dT];
    
        // Compute coefficients
        util.mmult(coeffs, *aeroMatrix, inputs, 8, 11);
    
        // Lift and drag coefficients to body frame
        double liftDragWind[3] = {coeffs[0], 0, coeffs[1]};
        double liftDragBody[3];
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
        util.vgain(aeroForce, -1.0, 3);
    }
}

// **********************************************************************
// Simple RC Aerodynamic Model
// **********************************************************************
SimpleRCAeroModel::SimpleRCAeroModel(ModelMap *pMapInit, bool debugFlagIn) : AeroModelBase(pMapInit, debugFlagIn)
{
    refArea = wingArea;
    
    dCd_dCL  = 0; // computed dynamically
    Cdarray[constant]  = 0.04;        // Cdo
    Cdarray[dalpha]    = 0.0;         // Cda  computed dynamically
    Cdarray[dbq]       = 0.0;         // Cdq  computed dynamically
    Cdarray[delevator] = 0.0;         // Cdde computed dynamically
    
    CLarray[constant]  = 0.31;        // CLo
    CLarray[dalpha]    = 4.2072;      // CLa
    CLarray[dbq]       = 6.7408;      // CLq
    CLarray[delevator] = 0.50583;     // CLde
    
    Cmarray[constant]  = 0.02221337;  // Cmo
    Cmarray[dalpha]    = -0.82203;    // Cma
    Cmarray[dbq]       = -8.9323;     // Cmq
    Cmarray[delevator] = -1.1669;     // Cmde
    
    CYarray[dbeta]     = -0.35968;    // CYb
    CYarray[dbp]       = -0.00059002; // CYp
    CYarray[dbr]       = 0.40526;     // CYr
    CYarray[daileron]  = -0.13034;    // CYda
    CYarray[drudder]   = 0.28305;     // CYdr
    
    Clarray[dbeta]     = -0.04194;    // Clb
    Clarray[dbp]       = -0.39218;    // Clp
    Clarray[dbr]       = 0.15978;     // Clr
    Clarray[daileron]  = 0.40029;     // Clda
    Clarray[drudder]   = 0.0033462;   // Cldr
    
    Cnarray[dbeta]     = 0.18222;     // Cnb
    Cnarray[dbp]       = -0.063944;   // Cnp
    Cnarray[dbr]       = -0.20752;    // Cnr
    Cnarray[daileron]  = 0.065812;    // Cnda
    Cnarray[drudder]   = -0.15261;    // Cndr
}

void SimpleRCAeroModel::updateAeroCoeffs(void)
{
    // Compute CL
    double CLwing = CLarray[constant] + CLarray[dalpha]*alpha.rad();
    
    // Copmute derivative of Cd w.r.t. CL
    dCd_dCL = 2*CLwing/(M_PI*e*AR);
    
    // Update drag derivatives
    Cdarray[dalpha]    = CLarray[dalpha]    * dCd_dCL;
    Cdarray[dbq]       = CLarray[dbq]       * dCd_dCL;
    Cdarray[delevator] = CLarray[delevator] * dCd_dCL;
}

// **********************************************************************
// RC Airplane Aerodynamic Model
// **********************************************************************
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
    double Re = pAtmo->getRe();
    double elevator = actuators_init[0];
    double aileron = actuators_init[1];
    double rudder = actuators_init[2];
    
    // Drag coefficient lookup
    Cdarray[constant] = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCdo][0], Re, nReynolds);
    
    // Lift coefficient lookup
    CLarray[constant] = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLo][0], Re, nReynolds);
    CLarray[dalpha]   = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLa][0], Re, nReynolds);
    CLarray[dbq]      = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCLq][0], Re, nReynolds);
    
    // X Force coefficient lookup
    CXarray[du]        = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCXu][0], Re, nReynolds);
    CXarray[dalpha]    = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCXa][0], Re, nReynolds);
    CXarray[delevator] = util.interpolate(deVec, &elevatorTable[lookupCXde][0], elevator, nDe);
    CXarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCXda][0] , aileron , nDa);
    CXarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCXdr][0]  , rudder  , nDr);
    
    // Y (Side) Force coefficient lookup
    CYarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYb][0], Re, nReynolds);
    CYarray[dbp]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYp][0], Re, nReynolds);
    CYarray[dbr]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCYr][0], Re, nReynolds);
    CYarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCYda][0], Re, nReynolds);
    CYarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCYdr][0], Re, nReynolds);
    
    // Z Force coefficient lookup
    CZarray[du]        = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCZu][0], Re, nReynolds);
    CZarray[delevator] = util.interpolate(deVec, &elevatorTable[lookupCZde][0], elevator, nDe);
    CZarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCZda][0] , aileron , nDa);
    CZarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCZdr][0]  , rudder  , nDr);
    
    // Roll Moment coefficient lookup
    Clarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClb][0], Re, nReynolds);
    Clarray[dbp]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClp][0], Re, nReynolds);
    Clarray[dbr]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupClr][0], Re, nReynolds);
    Clarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupClda][0], Re, nReynolds);
    Clarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCldr][0], Re, nReynolds);
    
    // Pitch Moment coefficient lookup
    Cmarray[constant]  = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCmo][0], Re, nReynolds);
    Cmarray[dalpha]    = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCma][0], Re, nReynolds);
    Cmarray[dbq]       = util.interpolate(reynoldsVec, &AeroTable_Longitudinal[lookupCmq][0], Re, nReynolds);
    Cmarray[delevator] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCmde][0], Re, nReynolds);
    Cmarray[daileron]  = util.interpolate(daVec, &aileronTable[lookupCmda][0] , aileron , nDa);
    Cmarray[drudder]   = util.interpolate(drVec, &rudderTable[lookupCmdr][0]  , rudder  , nDr);
    
    // Rudder Moment coefficient lookup
    Cnarray[dbeta]    = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnb][0], Re, nReynolds);
    Cnarray[dbp]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnp][0], Re, nReynolds);
    Cnarray[dbr]      = util.interpolate(reynoldsVec, &AeroTable_Lateral[lookupCnr][0], Re, nReynolds);
    Cnarray[daileron] = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCnda][0], Re, nReynolds);
    Cnarray[drudder]  = util.interpolate(reynoldsVec, &AeroTable_Control[lookupCndr][0], Re, nReynolds);
}

void AeroModelBase::setAeroEuler(AngleType<double>* angles_in)
{
    for (int i = 0; i < 3; i++)
    {
        aeroEuler[i].val = angles_in[i].deg();
    }
    alpha.val = aeroEuler[1].rad();
    beta.val  = -aeroEuler[2].rad();
    pRotate->update();
}

// **********************************************************************
// Quadcopter Aerodynamic Model
// **********************************************************************
QuadcopterAeroModel::QuadcopterAeroModel(ModelMap *pMapInit, bool debugFlagIn) : AeroModelBase(pMapInit, debugFlagIn)
{
    cdo = 0.05;
    refArea = quadTopArea;
    
    Cdarray[constant]  = cdo;  // Cdo
    Cdarray[dalpha]    = 0.0;  // Cda
    Cdarray[dbq]       = 0.0;  // Cdq
    Cdarray[delevator] = 0.0;  // Cdde
    
    CLarray[constant]  = 0.0;  // CLo
    CLarray[dalpha]    = 0.0;  // CLa
    CLarray[dbq]       = 0.0;  // CLq
    CLarray[delevator] = 0.0;  // CLde
    
    Cmarray[constant]  = 0.0;  // Cmo
    Cmarray[dalpha]    = 0.0;  // Cma
    Cmarray[dbq]       = 0.0;  // Cmq
    Cmarray[delevator] = 0.0;  // Cmde
    
    CYarray[dbeta]     = 0.0;  // CYb
    CYarray[dbp]       = 0.0;  // CYp
    CYarray[dbr]       = 0.0;  // CYr
    CYarray[daileron]  = 0.0;  // CYda
    CYarray[drudder]   = 0.0;  // CYdr
    
    Clarray[dbeta]     = 0.0;  // Clb
    Clarray[dbp]       = 0.0;  // Clp
    Clarray[dbr]       = 0.0;  // Clr
    Clarray[daileron]  = 0.0;  // Clda
    Clarray[drudder]   = 0.0;  // Cldr
    
    Cnarray[dbeta]     = 0.0;  // Cnb
    Cnarray[dbp]       = 0.0;  // Cnp
    Cnarray[dbr]       = 0.0;  // Cnr
    Cnarray[daileron]  = 0.0;  // Cnda
    Cnarray[drudder]   = 0.0;  // Cndr
}

void QuadcopterAeroModel::updateAeroCoeffs(void)
{
    // Possibily a function of attitude
    Cdarray[constant]  = cdo;  // Cdo
}
