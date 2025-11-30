//
//  fs_thrust_estimator.cpp
//  Aircraft_Simulation
//
//  Created by Alex McLean on 9/16/23.
//  Copyright © 2023 Alexander McLean. All rights reserved.
//

#include "fs_thrust_estimator.hpp"
#include "fs_controls.hpp"
#include "fs_navigation.hpp"
#ifdef SIMULATION
    #include "dynamics_model.hpp"
    #include "propulsion_model.hpp"
#endif

ThrustEstimatorType thrustEstimateData;
#ifdef SIMULATION
ThrustEstimatorType thrustTruthData;
ThrustEstimatorType thrustEstimateError;
#endif

bool ThrustEstimator_setup = false;
double pwmSlope;
double pwmOffset;
const double* gB;
double vB[3];
double KT_L_est;
double KH_est;
const double* rpm;
const double* w;
double rpssqr[4];
double w_cross_Iw[3];
double w_cross_v[3];
double A;
double B;

// Simulation Models
#ifdef SIMULATION
    class DynamicsModel*       pDynamics = 0;
    class PropulsionModelBase* pPropulsion = 0;
#endif

// Setup
void FsThrustEstimator_setup()
{
#ifdef THRUST_ESTIMATOR
    pwmSlope  = (MAXRPM/(rpm2rps)) / (PWMMAX-PWMMIN);
    pwmOffset = -pwmSlope*PWMMIN;
    
    gB = NULL;
    memset(vB, 0, sizeof(vB));
    
    KT_L_est = 0.0;
    KH_est = 0.0;
    rpm = &thrustEstimateData.motorRPM[0];
    w   = &thrustEstimateData.bodyRate[0];
    memset(rpssqr, 0, sizeof(rpssqr));
    memset(w_cross_Iw, 0, sizeof(w_cross_Iw));
    memset(w_cross_v, 0, sizeof(w_cross_v));
    
    thrustEstimateData.tau_est = 0.008;
    thrustEstimateData.KT_est  = KT;
    thrustEstimateData.torqueRatio_est = TorqueRatio;
    
    reset_delta();
#endif
}

// Perform
void rpmUpdate(double &dt)
{
#ifdef THRUST_ESTIMATOR
    if (thrustEstimateData.tau_est != 0.0) { A = exp(-dt/thrustEstimateData.tau_est); }
    else                                   { A = 0.0; }
    B = (1.0 - A);
    
    for (int i = 0; i < 4; i++)
    {
        thrustEstimateData.motorRPM[i] = A*thrustEstimateData.motorRPM[i] + B*thrustEstimateData.motorRPMcmd[i];
    }
#endif
}

void FsThrustEstimator_perform(double &dt)
{
#ifdef THRUST_ESTIMATOR
    if (!ThrustEstimator_setup)
    {
        if (gB != NULL && vB != NULL)
        {
            ThrustEstimator_setup = true;
        }
    }
    if (!ThrustEstimator_setup) { return; }
    
    rpmUpdate(dt);
    update_variables();
    thrust_update();
    state_update(dt);
    variance_update(dt);
    
#ifdef SIMULATION
    update_truth_errors();
#endif
#endif
}

void update_variables()
{
    // short hand
    KT_L_est = thrustEstimateData.KT_est * (PropDistance);
    KH_est = thrustEstimateData.torqueRatio_est * thrustEstimateData.KT_est;
    for (int i = 0; i < 4; i++) {rpssqr[i] = rpm[i]*rpm[i]*(rpm2rps)*(rpm2rps); }
    w_cross_Iw[0] = -w[1]*w[2]*((Izz) - (Iyy));
    w_cross_Iw[1] = -w[0]*w[2]*((Ixx) - (Izz));
    w_cross_Iw[2] = -w[0]*w[1]*((Iyy) - (Ixx));
    
    //accBody = (F - bodyRates x m*velBody) / mass
    crossProduct(w_cross_v, w, vB);
}

void thrust_update()
{
    for (int i = 0; i < 4; i++)
    {
        thrustEstimateData.motorThrustEst[i] = thrustEstimateData.KT_est * rpssqr[i];
    }
    
    thrustEstimateData.bodyThrustEst[0] = 0.0;
    thrustEstimateData.bodyThrustEst[1] = 0.0;
    thrustEstimateData.bodyThrustEst[2] = -(thrustEstimateData.motorThrustEst[0] + thrustEstimateData.motorThrustEst[1] + thrustEstimateData.motorThrustEst[2] + thrustEstimateData.motorThrustEst[3]);
    
    thrustEstimateData.bodyMomentEst[0] = KT_L_est * (rpssqr[0] - rpssqr[1] - rpssqr[2] + rpssqr[3]);
    thrustEstimateData.bodyMomentEst[1] = KT_L_est * (rpssqr[0] + rpssqr[1] - rpssqr[2] - rpssqr[3]);
    thrustEstimateData.bodyMomentEst[2] = KH_est   * (rpssqr[0] - rpssqr[1] + rpssqr[2] - rpssqr[3]);
}

void state_update(double &dt)
{
    // Acceleration estimates
    
    // bodyAngularAcc = I^-1 * (M - bodyRates x I*bodyRates)
    thrustEstimateData.bodyAngAccel[0] = (thrustEstimateData.bodyMomentEst[0] + w_cross_Iw[0])/(Ixx);
    thrustEstimateData.bodyAngAccel[1] = (thrustEstimateData.bodyMomentEst[1] + w_cross_Iw[1])/(Iyy);
    thrustEstimateData.bodyAngAccel[2] = (thrustEstimateData.bodyMomentEst[2] + w_cross_Iw[2])/(Izz);
    
    //accBody = (F - bodyRates x m*velBody) / mass
    if (!FsControls_onGround())
    {
        thrustEstimateData.az = gB[2] + thrustEstimateData.bodyThrustEst[2]/(Mass);
    }
    
    // state integration
    for (int i = 0; i < 3; i++)
    {
        thrustEstimateData.deltaAngle[i] += thrustEstimateData.bodyRate[i]*dt + 0.5*thrustEstimateData.bodyAngAccel[i]*dt*dt;
        thrustEstimateData.bodyRate[i]   += thrustEstimateData.bodyAngAccel[i]*dt;
    }
    thrustEstimateData.deltaVz += thrustEstimateData.az*dt;
}

void variance_update(double &dt)
{

}

#ifdef SIMULATION
void update_truth_errors()
{
    // Update Truth
    thrustTruthData.deltaAngle[0] = pDynamics->getDeltaTheta()[0];
    thrustTruthData.deltaAngle[1] = pDynamics->getDeltaTheta()[1];
    thrustTruthData.deltaAngle[2] = pDynamics->getDeltaTheta()[2];
    
    thrustTruthData.bodyRate[0] = pDynamics->getBodyRates()[0];
    thrustTruthData.bodyRate[1] = pDynamics->getBodyRates()[1];
    thrustTruthData.bodyRate[2] = pDynamics->getBodyRates()[2];

    thrustTruthData.bodyAngAccel[0] = pDynamics->getbodyAngularAcc()[0];
    thrustTruthData.bodyAngAccel[1] = pDynamics->getbodyAngularAcc()[1];
    thrustTruthData.bodyAngAccel[2] = pDynamics->getbodyAngularAcc()[2];
    
    thrustTruthData.deltaVz = pDynamics->getDeltaVelocity()[2];
    thrustTruthData.az = pDynamics->getAccBody()[2];
    
    thrustTruthData.bodyThrustEst[0] = pPropulsion->getForce()[0];
    thrustTruthData.bodyThrustEst[1] = pPropulsion->getForce()[1];
    thrustTruthData.bodyThrustEst[2] = pPropulsion->getForce()[2];
    
    thrustTruthData.bodyMomentEst[0] = pPropulsion->getMoment()[0];
    thrustTruthData.bodyMomentEst[1] = pPropulsion->getMoment()[1];
    thrustTruthData.bodyMomentEst[2] = pPropulsion->getMoment()[2];
    
    thrustTruthData.motorThrustEst[0] = -pPropulsion->getPropulsors(0)->getForce()[2];
    thrustTruthData.motorThrustEst[1] = -pPropulsion->getPropulsors(1)->getForce()[2];
    thrustTruthData.motorThrustEst[2] = -pPropulsion->getPropulsors(2)->getForce()[2];
    thrustTruthData.motorThrustEst[3] = -pPropulsion->getPropulsors(3)->getForce()[2];
    
    thrustTruthData.motorRPM[0] = pPropulsion->getPropulsors(0)->getRPM();
    thrustTruthData.motorRPM[1] = pPropulsion->getPropulsors(1)->getRPM();
    thrustTruthData.motorRPM[2] = pPropulsion->getPropulsors(2)->getRPM();
    thrustTruthData.motorRPM[3] = pPropulsion->getPropulsors(3)->getRPM();

    thrustTruthData.tau_est         = pPropulsion->getTimeConstant();
    thrustTruthData.KT_est          = pPropulsion->getPropulsors(0)->getThrustCoeff();
    thrustTruthData.torqueRatio_est = pPropulsion->getPropulsors(0)->getTorqueRatio();
    
    // Update Errors
    for (int i = 0; i < 3; i++)
    {
        thrustEstimateError.deltaAngle[i]   = (thrustEstimateData.deltaAngle[i] - thrustTruthData.deltaAngle[i]) * radian2degree;
        thrustEstimateError.bodyRate[i]     = (thrustEstimateData.bodyRate[i] - thrustTruthData.bodyRate[i]) * radian2degree;
        thrustEstimateError.bodyAngAccel[i] = (thrustEstimateData.bodyAngAccel[i] - thrustTruthData.bodyAngAccel[i]) * radian2degree;
        
        thrustEstimateError.bodyThrustEst[i] = thrustEstimateData.bodyThrustEst[i] - thrustTruthData.bodyThrustEst[i];
        thrustEstimateError.bodyMomentEst[i] = thrustEstimateData.bodyMomentEst[i] - thrustTruthData.bodyMomentEst[i];
    }
    
    for (int i = 0; i < 4; i++)
    {
        thrustEstimateError.motorThrustEst[i] = thrustEstimateData.motorThrustEst[i] - thrustTruthData.motorThrustEst[i];
        thrustEstimateError.motorRPM[i] = thrustEstimateData.motorRPM[i] - thrustTruthData.motorRPM[i];
    }
    
    thrustEstimateError.deltaVz         = thrustEstimateData.deltaVz - thrustTruthData.deltaVz;
    thrustEstimateError.az              = thrustEstimateData.az - thrustTruthData.az;
    thrustEstimateError.tau_est         = thrustEstimateData.tau_est - thrustTruthData.tau_est;
    thrustEstimateError.KT_est          = thrustEstimateData.KT_est - thrustTruthData.KT_est;
    thrustEstimateError.torqueRatio_est = thrustEstimateData.torqueRatio_est - thrustTruthData.torqueRatio_est;
}
#endif

void FsThrustEstimator_setMotorPWMCmd(const double* pwmCmd)
{
    thrustEstimateData.motorRPMcmd[0] = pwmToRPM(pwmCmd[0]);
    thrustEstimateData.motorRPMcmd[1] = pwmToRPM(pwmCmd[1]);
    thrustEstimateData.motorRPMcmd[2] = pwmToRPM(pwmCmd[2]);
    thrustEstimateData.motorRPMcmd[3] = pwmToRPM(pwmCmd[3]);
}

void FsThrustEstimator_setGravity(const double* gravity)
{
    gB = gravity;
}
 
void FsThrustEstimator_setVelocity(const double* velocityNED)
{
    FsNavigation_NEDToBody(vB, velocityNED);
}

#ifdef SIMULATION
void FsThrustEstimator_setSimulationModels(ModelMap* pMap)
{
    pDynamics   = (DynamicsModel*)       pMap->getModel("DynamicsModel");
    pPropulsion = (PropulsionModelBase*) pMap->getModel("PropulsionModel");
}
#endif

void FsThrustEstimator_performIMUupdate(const double* imuDeltaAngle, const double * imuDeltaVelocity)
{
#ifdef THRUST_ESTIMATOR
    for (int i=0; i<3; i++)
    {
        thrustEstimateData.imuResidual[i] = (imuDeltaAngle[i] - thrustEstimateData.deltaAngle[i]) / (degree2radian);
    }
    thrustEstimateData.imuResidual[3] = imuDeltaVelocity[2] - thrustEstimateData.deltaVz;
    reset_delta();
#endif
}

const ThrustEstimatorType* FsThrustEstimator_getThrustEstimatorData()
{
    return &thrustEstimateData;
}

#ifdef SIMULATION
const ThrustEstimatorType* FsThrustEstimator_getThrustEstimatorError()
{
    return &thrustEstimateError;
}
#endif

void reset_delta()
{
    for (int i = 0; i < 3; i++)
    {
        thrustEstimateData.deltaAngle[i] = 0.0;
        //thrustEstimateData.bodyRate[i]   = 0.0;
    }
    thrustEstimateData.deltaVz = 0.0;
}

double pwmToRPM(double pwm)
{
    double rpm = pwmSlope * pwm + pwmOffset;
    if (rpm < MINRPM/(rpm2rps)) { rpm = 0.0; }
    return rpm;
}
