//
//  propulsion_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "actuator_model.hpp"
#include "atmosphere_model.hpp"
#include "time.hpp"

#include "propulsion_model.hpp"

// **********************************************************************
// Propulsion Model Base
// **********************************************************************
PropulsionModelBase::PropulsionModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pAct    = NULL;
    pTime   = NULL;
    pMap    = pMapInit;
    
    for (int i=0; i<maxPropulsors; i++)
    {
        pPropulsors[i] = NULL;
    }
}

PropulsionModelBase::~PropulsionModelBase(void)
{
    for (int i=0; i<maxPropulsors; i++)
    {
        if (pPropulsors[i] != NULL)
        {
            delete pPropulsors[i];
        }
    }
}

void PropulsionModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*)     pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)       pMap->getModel("RotateFrame");
    pAct    = (ActuatorModelBase*) pMap->getModel("ActuatorModel");
    pTime   = (Time*)              pMap->getModel("Time");
    
    for (int i=0; i<maxPropulsors; i++)
    {
        if (pPropulsors[i] != NULL)
        {
            pPropulsors[i]->initialize();
        }
    }
}

bool PropulsionModelBase::update(void)
{
    util.setArray(LLForce, zero_init, 3);
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
    
    // Update throttle, get RPM, etc. for each propulsor
    updatePropulsorStates();
    
    // Comput forces and moments
    for (int i=0; i<maxPropulsors; i++)
    {
        if (pPropulsors[i] != NULL)
        {
            pPropulsors[i]->update();
            util.vAdd(bodyForce , bodyForce , pPropulsors[i]->getForce()  , 3);
            util.vAdd(LLForce   , LLForce   , pPropulsors[i]->getLLForce(), 3);
            util.vAdd(bodyMoment, bodyMoment, pPropulsors[i]->getMoment() , 3);
        }
    }
    
    // Account for numerical errors
    for (int i=0;i<3;i++)
    {
        if (fabs(bodyForce[i])  < util.zeroTolerance) { bodyForce[i] = 0.0; }
        if (fabs(bodyMoment[i]) < util.zeroTolerance) { bodyMoment[i] = 0.0; }
    }
    
    if (debugFlag)
    {
        printf("Propulsion Model Update:\n");
        util.print(bodyForce, 3, "bodyForce");
        util.print(LLForce, 3, "LLForce");
        util.print(bodyMoment, 3, "bodyMoment");
    }
    return true;
}

void PropulsionModelBase::updatePropulsorStates(void)
{
    // Complete in derived class
}

// **********************************************************************
// RC Plane Propulsion Model
// **********************************************************************
RCPropulsionModel::RCPropulsionModel(ModelMap *pMapInit, bool debugFlagIn) : PropulsionModelBase(pMapInit, debugFlagIn)
{
    pMap = pMapInit;
    
    debugFlag = debugFlagIn;

    //pMap->addLogVar("throttle", &throttle, printSavePlot, 3);
    pMap->addLogVar("Thrust", &bodyForce[0], printSavePlot, 3);
    pMap->addLogVar("engineRPM", &engineRPM, printSavePlot, 3);
    //pMap->addLogVar("propLLX", &LLForce[0], savePlot, 2);
    
    throttle = 0.0;
    engineRPM = 0.0;
    
    pPropulsors[0] = new Propeller(pMap, debugFlag, maxThrust, maxRPM);
    pPropulsors[0]->setTorqueRatio(0.0);
    pPropulsors[0]->setLocation((double*) location);
    pPropulsors[0]->setOrientation((double*) orientation);
    static_cast<Propeller*>(pPropulsors[0])->setTimeConstant(0.1);
}

void RCPropulsionModel::initialize(void)
{
    Base::initialize();
    pPropulsors[0]->setThrottle( pAct->getActuators()[RCActuatorModel::dT] );
    pPropulsors[0]->initialize();
}

void RCPropulsionModel::updatePropulsorStates(void)
{
    throttle = pAct->getActuators()[RCActuatorModel::dT];
    pPropulsors[0]->setThrottle( throttle );
    engineRPM = pPropulsors[0]->getRPM();
}

// **********************************************************************
// Quadcopter Propulsion Model
// **********************************************************************
QuadcopterPropulsionModel::QuadcopterPropulsionModel(ModelMap *pMapInit, bool debugFlagIn) : PropulsionModelBase(pMapInit, debugFlagIn)
{
    pMap = pMapInit;
    
    debugFlag = debugFlagIn;
    /*
    pMap->addLogVar("propX", &bodyForce[0], savePlot, 2);
    pMap->addLogVar("propY", &bodyForce[1], savePlot, 2);
    pMap->addLogVar("propZ", &bodyForce[2], savePlot, 2);
    pMap->addLogVar("propMX", &bodyMoment[0], savePlot, 2);
    pMap->addLogVar("propMY", &bodyMoment[1], savePlot, 2);
    pMap->addLogVar("propMZ", &bodyMoment[2], savePlot, 2);
    //pMap->addLogVar("prop T1", &throttle[0], savePlot, 2);
    //pMap->addLogVar("prop T2", &throttle[1], savePlot, 2);
    //pMap->addLogVar("prop T3", &throttle[2], savePlot, 2);
    //pMap->addLogVar("prop T4", &throttle[3], savePlot, 2);
    pMap->addLogVar("engineRPM1", &engineRPM[0], savePlot, 2);
    pMap->addLogVar("engineRPM2", &engineRPM[1], savePlot, 2);
    pMap->addLogVar("engineRPM3", &engineRPM[2], savePlot, 2);
    pMap->addLogVar("engineRPM4", &engineRPM[3], savePlot, 2);
    //pMap->addLogVar("propLLX", &LLForce[0], savePlot, 2);
 */
    pPropulsors[0] = new Propeller(pMap, debugFlag, maxThrust, maxRPM);
    pPropulsors[1] = new Propeller(pMap, debugFlag, maxThrust, maxRPM);
    pPropulsors[2] = new Propeller(pMap, debugFlag, maxThrust, maxRPM);
    pPropulsors[3] = new Propeller(pMap, debugFlag, maxThrust, maxRPM);
    
    for (int i=0; i<QuadcopterActuatorModel::nActuators; i++)
    {
        throttle[i]  = 0.0;
        engineRPM[i] = 0.0;
        pPropulsors[i]->setDirection(directions[i]);
        pPropulsors[i]->setLocation((double*) &locations[i]);
        pPropulsors[i]->setOrientation((double*) &attitudes[i]);
        static_cast<Propeller*>(pPropulsors[i])->setTimeConstant(riseTimes[i]);
    }
}

void QuadcopterPropulsionModel::initialize(void)
{
    Base::initialize();
    for (int i=0; i<QuadcopterActuatorModel::nActuators; i++)
    {
        pPropulsors[i]->setThrottle( pAct->getActuators()[i] );
        pPropulsors[i]->initialize();
    }
}

void QuadcopterPropulsionModel::updatePropulsorStates(void)
{
    for (int i=0; i<QuadcopterActuatorModel::nActuators; i++)
    {
        throttle[i] = pAct->getActuators()[i];
        pPropulsors[i]->setThrottle( throttle[i] );
        engineRPM[i] = pPropulsors[i]->getRPM();
    }
}

// **********************************************************************
// Propulsion Type Base Class
// **********************************************************************
PropulsionTypeBase::PropulsionTypeBase(ModelMap *pMapInit, bool debugFlagIn, double maxThrust_in, double maxRPM_in, int direction_in)
{
    pRotate = NULL;
    pAtmo   = NULL;
    pDyn    = NULL;
    pMap    = pMapInit;
    debugFlag = debugFlagIn;

    throttle  = 0.0;
    Q         = 0.003; // 0.024 - 0.036
    maxThrust = maxThrust_in;
    maxTorque = Q*maxThrust;
    maxRPM    = maxRPM_in;
    direction = direction_in;
    
    double rps2 = (maxRPM*util.rpm2rps)*(maxRPM*util.rpm2rps);
    b = maxThrust/rps2; //.005022
    k = maxTorque/rps2; //1.858e-5
    
    util.setArray(engineForce, zero_init, 3);
    util.setArray(engineMoment, zero_init, 3);
    util.setUnitClassArray(location, zero_init, meters, 3);
    util.setUnitClassArray(orientation, zero_init, degrees, 3);
}

void PropulsionTypeBase::initialize(void)
{
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pAtmo   = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    
    double maxRPSsquared = (maxRPM*util.rpm2rps)*(maxRPM*util.rpm2rps);
    
    maxTorque = maxThrust*Q;
    b = maxThrust/(maxRPSsquared);
    k = maxTorque/(maxRPSsquared);
    
    updateEngineRotations();
}

bool PropulsionTypeBase::update(void)
{
    updateEngineRotations();
    calculateForcesAndMoments();
    setBodyForcesAndMoments();
    pRotate->bodyToLL(LLForce, bodyForce);
    return true;
}

void PropulsionTypeBase::updateEngineRotations(void)
{
    util.eulerToQuaternion(qfEB, orientation);
    util.quaternionConjugate(qfBE, qfEB);
    
    util.eulerToQuaternion(qmEB, orientation);
    util.quaternionConjugate(qmBE, qmEB);
    
    util.setupRotation(*REB, orientation);
    util.mtran(*RBE, *REB, 3, 3);
    
    util.setupRotation(*TEB, orientation);
    util.mtran(*TBE, *TEB, 3, 3);
}

void PropulsionTypeBase::calculateForcesAndMoments(void)
{
    rpm = throttle*maxRPM;
    
    engineForce[0] = throttle*maxThrust;
    engineForce[1] = 0;
    engineForce[2] = 0;
    
    engineMoment[0] = 0.0; //throttle*direction*maxTorque;
    engineMoment[1] = 0.0;
    engineMoment[2] = 0.0;
}

void PropulsionTypeBase::setBodyForcesAndMoments(void)
{
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);

    util.quaternionTransformation(bodyForce, qfBE, engineForce);
    util.quaternionTransformation(bodyMoment, qmBE, engineMoment);

    if (debugFlag)
    {
        std::cout<<"PropulsionTypeBase: setBodyForcesAndMoments"<<std::endl;
        util.print(engineForce,3,"engineForce");
        util.print(engineMoment,3,"engineMoment");
        util.print(*RBE,3,3,"RBE");
        util.print(*TBE,3,3,"TBE");
        util.print(qfBE,4,"qfBE");
        util.print(qmBE,4,"qmBE");
    }
    
    double torque[3];
    util.crossProduct(torque, location, bodyForce);
    
    if (debugFlag)
    {
        util.print(bodyForce,3,"bodyForce");
        util.print(bodyMoment,3,"bodyMoment");
        util.print(location,3,"location");
        util.print(torque,3,"torques");
    }
    
    util.vAdd(bodyMoment, bodyMoment, torque, 3);
    
    if (debugFlag)
    {
        util.print(bodyMoment,3,"bodyMoment+torques");
    }
}

// **********************************************************************
// Simple Thrust Model
// **********************************************************************
SimpleThrustEngine::SimpleThrustEngine(ModelMap *pMapInit, bool debugFlagIn, double maxThrust_in, double maxRPM_in, int direction_in) : PropulsionTypeBase(pMapInit, debugFlagIn, maxThrust_in, maxRPM_in, direction_in)
{
    pMap = pMapInit;
    debugFlag = debugFlagIn;
}

// **********************************************************************
// Propeller Model
// **********************************************************************
Propeller::Propeller(ModelMap *pMapInit, bool debugFlagIn, double maxThrust_in, double maxRPM_in, int direction_in) : PropulsionTypeBase(pMapInit, debugFlagIn, maxThrust_in, maxRPM_in, direction_in)
{
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    throttleController.setGain(maxRPM);
}

void Propeller::initialize(void)
{
    Base::initialize();
    throttleController.setInitialValue( throttle );
    throttleController.setDt( dynamicsInterval_init );
}

void Propeller::calculateForcesAndMoments(void)
{
    // Throttle creates RPM response
    throttleController.setValue(throttle);
    rpm = throttleController.getFilterValue();
    
    double rps2 = (rpm*util.rpm2rps)*(rpm*util.rpm2rps);

    // Forces
    engineForce[0] = b*rps2;
    engineForce[1] = 0.0;
    engineForce[2] = 0.0;
    
    // Moments
    engineMoment[0] = direction*k*rps2;
    engineMoment[1] = 0.0;
    engineMoment[2] = 0.0;
}
