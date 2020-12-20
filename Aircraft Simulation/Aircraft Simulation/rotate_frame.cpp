//
//  Coordinate Transformation.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/17/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//
// Rotates a 3D vector
//      RotateInertialToBody - Rotates a vector from inertial frame to body frame representation
//      RotateBodyToInertial - Rotates a vector from body frame to inertial frame representation

#include "initial_conditions.hpp"
#include "dynamics_model.hpp"
#include "aero_model.hpp"
#include "model_mapping.hpp"

#include "rotate_frame.hpp"

RotateFrame::RotateFrame(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn  = NULL;
    pAero = NULL;
    pMap  = pMapInit;
    
    // IMU frame
    util.setUnitClassArray(imuFrame, imuFrame_init, degrees, 3);
    util.setupRotation(*T_B_imu, imuFrame);
    util.mtran(*T_imu_B, *T_B_imu, 3, 3);
    util.setUnitClassUnit(imuFrame, radians, 3);
    
    // Body rotations
    util.setUnitClassArray(eulerAngles, eulerAngles_init, degrees, 3);
    
    util.eulerToQuaternion(q_B_NED, eulerAngles);
    util.quaternionConjugate(q_NED_B, q_B_NED);
    util.setArray(q_B_LL, quaternion_init, 4);
    util.setArray(q_LL_B, quaternion_init, 4);
    
    util.setupRotation(*R_B_NED, eulerAngles);
    util.mtran(*R_NED_B, *R_B_NED, 3, 3);
    util.setMatrix(*R_B_LL, *identityMatrix, 3, 3);
    util.setMatrix(*R_LL_B, *identityMatrix, 3, 3);
    
    // Wind rotations
    util.setMatrix(*R_B_W, *identityMatrix, 3, 3);
    util.setMatrix(*R_W_B, *identityMatrix, 3, 3);
    
    // Angular rate transformations
    util.setupEulerRateToBodyRate(*L_B_E, eulerAngles);
    util.setupBodyRateToEulerRate(*L_E_B, eulerAngles);
    
    debugFlag = debugFlagIn;
    
    if (debugFlag)
    {
        printf("Rotate Frame Constructor:\n");
        util.print(*T_B_imu,3,3,"T_B_imu:");
        util.print(q_B_NED,4,"q_B_NED:");
        util.print(*R_B_NED,3,3,"R_B_NED:");
        util.print(*R_B_W,3,3,"R_B_W:");
        util.print(*L_B_E,3,3,"L_B_E:");
    }
}

void RotateFrame::initialize()
{
    pDyn = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pAero = (AeroModelBase*) pMap->getModel("AeroModel");
    
    // Update NED to Body rotation from dynamics model
    util.setArray(q_B_NED, pDyn->get_q_B_NED(), 4);
    util.quaternionConjugate(q_NED_B, q_B_NED);
    
    util.setupRotation(*R_B_NED, pDyn->getEulerAngles() );
    util.mtran(*R_NED_B, *R_B_NED, 3, 3);
    
    // Update Local Level to Body rotation
    util.setArray(q_B_LL, pDyn->get_q_B_LL(), 4);
    util.quaternionConjugate(q_LL_B, q_B_LL);
    
    double eulerLL[3];
    util.setArray(eulerLL, pDyn->getEulerAngles(), 3 );
    eulerLL[2] = 0;
    util.setupRotation(*R_B_LL, eulerLL);
    util.mtran(*R_LL_B, *R_B_LL, 3, 3);
    
    // Update wind to body rotation from aero model
    util.setupRotation(*R_B_W, pAero->getAeroEuler() );
    util.mtran(*R_W_B, *R_B_W, 3, 3);
    
    // Update angle rate matrices
    util.setupEulerRateToBodyRate(*L_B_E, pDyn->getEulerAngles() );
    util.setupBodyRateToEulerRate(*L_E_B, pDyn->getEulerAngles() );
    
    if (debugFlag)
    {
        printf("Rotate Frame Init:\n");
        util.print(q_B_NED,4,"q_B_NED:");
        util.print(*R_B_NED,3,3,"R_B_NED:");
        util.print(*R_B_W,3,3,"R_B_W:");
        util.print(*L_B_E,3,3,"L_B_E:");
    }
}

bool RotateFrame::update(void)
{
    double eulerLL[3];
    
    // Update NED to Body rotation from dynamics model
    util.setArray(q_B_NED, pDyn->get_q_B_NED(), 4);
    util.quaternionConjugate(q_NED_B, q_B_NED);
    
    util.setupRotation(*R_B_NED, pDyn->getEulerAngles() );
    util.mtran(*R_NED_B, *R_B_NED, 3, 3);
    
    // Update Local Level to Body rotation
    util.setArray(q_B_LL, pDyn->get_q_B_LL(), 4);
    util.quaternionConjugate(q_LL_B, q_B_LL);
    
    util.setArray(eulerLL, pDyn->getEulerAngles(), 3);
    util.vgain(eulerLL, (double)(1/util.deg2rad), 3);
    
    eulerLL[2] = 0;
    util.setupRotation(*R_B_LL, eulerLL);
    util.mtran(*R_LL_B, *R_B_LL, 3, 3);
    
    // Update wind to body rotation from aero model
    util.setupRotation(*R_B_W, pAero->getAeroEuler() );
    util.mtran(*R_W_B, *R_B_W, 3, 3);
    
    // Update angle rate matrices
    util.setupEulerRateToBodyRate(*L_B_E, pDyn->getEulerAngles() );
    util.setupBodyRateToEulerRate(*L_E_B, pDyn->getEulerAngles() );
    
    if (debugFlag)
    {
        printf("Rotate Frame Update:\n");
        util.print(q_B_NED,4,"q_B_NED:");
        util.print(*R_B_NED,3,3,"R_B_NED:");
        util.print(*R_B_W,3,3,"R_B_W:");
        util.print(*L_B_E,3,3,"L_B_E:");
    }
    return true;
}

// NED
void RotateFrame::bodyToNED(double *NEDFrame, double *bodyFrame)
{
    util.quaternionTransformation(NEDFrame, q_NED_B, bodyFrame);
}

template<typename valType, template<typename T> class unitType>
void RotateFrame::bodyToNED(unitType<valType> *NEDFrame, unitType<valType> *bodyFrame)
{
    util.quaternionTransformation(NEDFrame, q_NED_B, bodyFrame);
}

void RotateFrame::NEDToBody(double *bodyFrame, double *NEDFrame)
{
    util.quaternionTransformation(bodyFrame, q_B_NED, NEDFrame);
}

template<typename valType, template<typename T> class unitType>
void RotateFrame::NEDToBody(unitType<valType> *bodyFrame, unitType<valType> *NEDFrame)
{
    util.quaternionTransformation(bodyFrame, q_B_NED, NEDFrame);
}

// Local Level
void RotateFrame::bodyToLL(double *LLFrame, double *bodyFrame)
{
    util.quaternionTransformation(LLFrame, q_LL_B, bodyFrame);
    //util.mmult(LLFrame, *R_LL_B, bodyFrame, 3, 3);
}

template<typename valType, template<typename T> class unitType>
void RotateFrame::bodyToLL(unitType<valType> *LLFrame, unitType<valType> *bodyFrame)
{
    util.quaternionTransformation(LLFrame, q_LL_B, bodyFrame);
    //util.mmult(LLFrame, *R_LL_B, bodyFrame, 3, 3);
}

void RotateFrame::LLToBody(double *bodyFrame, double *LLFrame)
{
    util.quaternionTransformation(bodyFrame, q_B_LL, LLFrame);
    //util.mmult(bodyFrame, *R_B_LL, LLFrame, 3, 3);
}

template<typename valType, template<typename T> class unitType>
void RotateFrame::LLToBody(unitType<valType> *bodyFrame, unitType<valType> *LLFrame)
{
    util.quaternionTransformation(bodyFrame, q_B_LL, LLFrame);
    //util.mmult(bodyFrame, *R_B_LL, LLFrame, 3, 3);
}

// Sensors
void RotateFrame::imuToBody(double *bodyFrame, double *imuFrame)
{
    util.mmult(bodyFrame, *T_B_imu, imuFrame, 3, 3);
}

void RotateFrame::bodyToImu(double *imuFrame, double *bodyFrame)
{
    util.mmult(imuFrame, *T_imu_B, bodyFrame, 3, 3);
}

// Wind
void RotateFrame::windToBody(double *bodyFrame, double *windFrame)
{
    util.mmult(bodyFrame, *R_B_W, windFrame, 3, 3);
}

void RotateFrame::bodyToWind(double *windFrame, double *bodyFrame)
{
    util.mmult(windFrame, *R_W_B, bodyFrame, 3, 3);
}

// Angle Rates
void RotateFrame::eulerRateToBodyRate(AngleRateType<double> *bodyRates, AngleRateType<double> *eulerRates)
{
    util.mmult(bodyRates, *L_B_E, eulerRates, 3, 3);
}

void RotateFrame::bodyRateToEulerRate(AngleRateType<double> *eulerRates, AngleRateType<double> *bodyRates)
{
    util.mmult(eulerRates, *L_E_B, bodyRates, 3, 3);
}

// Template Definitions
template void RotateFrame::bodyToNED(DistanceType<double>*, DistanceType<double>*);
template void RotateFrame::bodyToNED(SpeedType<double>*, SpeedType<double>*);

template void RotateFrame::NEDToBody(DistanceType<double>*, DistanceType<double>*);
template void RotateFrame::NEDToBody(SpeedType<double>*, SpeedType<double>*);

template void RotateFrame::bodyToLL(DistanceType<double>*, DistanceType<double>*);
template void RotateFrame::bodyToLL(SpeedType<double>*, SpeedType<double>*);

template void RotateFrame::LLToBody(DistanceType<double>*, DistanceType<double>*);
template void RotateFrame::LLToBody(SpeedType<double>*, SpeedType<double>*);
