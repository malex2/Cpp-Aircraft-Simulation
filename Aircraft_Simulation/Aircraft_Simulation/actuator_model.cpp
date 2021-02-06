//
//  actuator_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include <cstdio>
#include <iostream>

#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "time.hpp"
#include "propulsion_model.hpp"
#include "aero_model.hpp"
#include "atmosphere_model.hpp"

#include "actuator_model.hpp"

// **********************************************************************
// Actuator Model Base
// **********************************************************************
ActuatorModelBase::ActuatorModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pProp   = NULL;
    pAero   = NULL;
    pAtmo   = NULL;
    pMap    = pMapInit;
    
    pTerminal = new MonitorMacInput;
    
    for (int i=0; i<maxActuators; i++)
    {
        positions[i] = 0.0;
        commands[i]  = 0.0;
        actuators[i] = NULL;
        pTable[i]    = NULL;
    }
    
    std::fill_n(continueArray, maxActuators, true);
}

ActuatorModelBase::~ActuatorModelBase(void)
{
    for (int i=0; i<maxActuators; i++)
    {
        if (actuators[i] != NULL) { delete actuators[i]; }
    }
    delete pTerminal;
}

void ActuatorModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*)       pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)         pMap->getModel("RotateFrame");
    pTime   = (Time*)                pMap->getModel("Time");
    pProp   = (PropulsionModelBase*) pMap->getModel("PropulsionModel");
    pAero   = (AeroModelBase*)       pMap->getModel("AeroModel");
    pAtmo   = (AtmosphereModel*)     pMap->getModel("AtmosphereModel");
    
    for (int i=0; i<maxActuators; i++)
    {
        if (actuators[i] != NULL)
        {
            actuators[i]->initialize();
        
            if (inputMode_init == ActuatorTypeBase::keyboard)
            {
                actuators[i]->setTerminal(pTerminal);
            }
        
            if (inputMode_init == ActuatorTypeBase::table)
            {
                if (pTable[i] != NULL)
                {
                    actuators[i]->setTable(pTable[i]);
                }
            }
        }
    }
}

bool ActuatorModelBase::update(void)
{
    for (int i = 0; i < maxActuators; i++)
    {
        if (actuators[i] != NULL)
        {
            actuators[i]->setCommand(commands[i]);
            continueArray[i] = actuators[i]->update();
            positions[i] = actuators[i]->getPosition();
            commands[i]  = actuators[i]->getCommand();
        }
    }
    
    return util.all(continueArray, maxActuators);
}

// **********************************************************************
// RC Plane Actuator Model
// **********************************************************************
RCActuatorModel::RCActuatorModel(ModelMap *pMapInit, bool debugFlagIn) : ActuatorModelBase(pMapInit, debugFlagIn)
{
    pMap      = pMapInit;
    debugFlag = debugFlagIn;

    //pMap->addLogVar("de cmd"      , &commands[de], printSave, 2);
    //pMap->addLogVar("da cmd"      , &commands[da], &save, 1);
    //pMap->addLogVar("dr cmd"      , &commands[dr], &save, 1);
    //pMap->addLogVar("tcmd   ", &commands[dT], &save, 2);
    
    //pMap->addLogVar("de  "    , &positions[de], printSavePlot, 3);
    //pMap->addLogVar("da   "   , &positions[da], printSavePlot, 3);
    //pMap->addLogVar("dr   "   , &positions[dr], printSavePlot, 3);
    //pMap->addLogVar("throttle", &positions[dT], printSavePlot, 3);
    
    // Create Actuators
    actuators[de] = new ServoMotor(pMap, debugFlag, actuators_init[de]);
    actuators[da] = new ServoMotor(pMap, debugFlag, actuators_init[da]);
    actuators[dr] = new ServoMotor(pMap, debugFlag, actuators_init[dr]);
    actuators[dT] = new BrushlessDCMotor(pMap, debugFlag, actuators_init[dT]);
    
    actuators[de]->setLimits(-90.0, 90.0);
    actuators[da]->setLimits(-90.0, 90.0);
    actuators[dr]->setLimits(-90.0, 90.0);
    actuators[dT]->setLimits(  0.0,  1.0);
    
    static_cast<ServoMotor*>(actuators[de])->setTimeConstant( servoTau[de] );
    static_cast<ServoMotor*>(actuators[da])->setTimeConstant( servoTau[da] );
    static_cast<ServoMotor*>(actuators[dr])->setTimeConstant( servoTau[dr] );
    
    static_cast<ServoMotor*>(actuators[de])->setDt( dynamicsInterval_init );
    static_cast<ServoMotor*>(actuators[da])->setDt( dynamicsInterval_init );
    static_cast<ServoMotor*>(actuators[dr])->setDt( dynamicsInterval_init );
  
    static_cast<ServoMotor*>(actuators[de])->setMaxRate( servoMaxRate[de] );
    static_cast<ServoMotor*>(actuators[da])->setMaxRate( servoMaxRate[da] );
    static_cast<ServoMotor*>(actuators[dr])->setMaxRate( servoMaxRate[dr] );
    
    static_cast<BrushlessDCMotor*>(actuators[dT])->setESCDelay(0.01);

    // Fill in Tables
    for (int i=0; i<nActuators;i++)
    {
        pTable[i] = new tableType;
        pTable[i]->pTableTimes  = &actuatorTimes[i][0];
        pTable[i]->pTableValues = &actuatorValues[i][0];
        pTable[i]->tableLength  = lengthTable;
    }
    
    //Terminal
    actuators[de]->setKeys(sKey, wKey);
    actuators[da]->setKeys(rightArrow, leftArrow);
    actuators[dr]->setKeys(dKey, aKey);
    actuators[dT]->setKeys(upArrow, downArrow);
    
    // Delays
    actuators[de]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[da]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[dr]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[dT]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    
    actuators[de]->setDelay(0.0, 0.2);
    actuators[da]->setDelay(0.0, 0.2);
    actuators[dr]->setDelay(0.0, 0.2);
    actuators[dT]->setDelay(0.0, 0.6);
    
    actuators[de]->setRate(1.0, 5.0); // deg/s
    actuators[da]->setRate(1.0, 5.0); // deg/s
    actuators[dr]->setRate(1.0, 5.0); // deg/s
    actuators[dT]->setRate(0.1, 0.3); // percent/s
}

// **********************************************************************
// Quadcopter Actuator Model
// **********************************************************************
QuadcopterActuatorModel::QuadcopterActuatorModel(ModelMap *pMapInit, bool debugFlagIn) : ActuatorModelBase(pMapInit, debugFlagIn)
{
    pMap      = pMapInit;
    debugFlag = debugFlagIn;
    
    pMap->addLogVar("T1 cmd", &commands[T1], &save, 1);
    pMap->addLogVar("T2 cmd", &commands[T2], &save, 1);
    pMap->addLogVar("T3 cmd", &commands[T3], &save, 1);
    pMap->addLogVar("T4 cmd", &commands[T4], &save, 1);
    
    pMap->addLogVar("T1 ", &positions[T1], &save, 1);
    pMap->addLogVar("T2 ", &positions[T2], &save, 1);
    pMap->addLogVar("T3 ", &positions[T3], &save, 1);
    pMap->addLogVar("T4 ", &positions[T4], &save, 1);
    
    // Create Actuators
    actuators[T1] = new BrushlessDCMotor(pMap, debugFlag, actuators_init[T1]);
    actuators[T2] = new BrushlessDCMotor(pMap, debugFlag, actuators_init[T2]);
    actuators[T3] = new BrushlessDCMotor(pMap, debugFlag, actuators_init[T3]);
    actuators[T4] = new BrushlessDCMotor(pMap, debugFlag, actuators_init[T4]);
    
    actuators[T1]->setLimits(0.0, 1.0);
    actuators[T2]->setLimits(0.0, 1.0);
    actuators[T3]->setLimits(0.0, 1.0);
    actuators[T4]->setLimits(0.0, 1.0);
    
    static_cast<BrushlessDCMotor*> (actuators[T1])->setESCDelay(escDelays[T1]);
    static_cast<BrushlessDCMotor*> (actuators[T2])->setESCDelay(escDelays[T2]);
    static_cast<BrushlessDCMotor*> (actuators[T3])->setESCDelay(escDelays[T3]);
    static_cast<BrushlessDCMotor*> (actuators[T4])->setESCDelay(escDelays[T4]);
    
    // Fill in Tables
    for (int i=0; i<nActuators;i++)
    {
        pTable[i] = new tableType;
        pTable[i]->pTableTimes  = &actuatorTimes[i][0];
        pTable[i]->pTableValues = &actuatorValues[i][0];
        pTable[i]->tableLength  = lengthTable;
    }
    
    //Terminal
    actuators[T1]->setKeys(sKey, wKey);
    actuators[T2]->setKeys(rightArrow, leftArrow);
    actuators[T3]->setKeys(dKey, aKey);
    actuators[T4]->setKeys(upArrow, downArrow);
    
    // Delays
    actuators[T1]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[T2]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[T3]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    actuators[T4]->setDebounce( new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0) );
    
    actuators[T1]->setDelay(0.0, 0.6);
    actuators[T2]->setDelay(0.0, 0.6);
    actuators[T3]->setDelay(0.0, 0.6);
    actuators[T4]->setDelay(0.0, 0.6);
    
    actuators[T1]->setRate(0.1, 0.3); // percent/s
    actuators[T2]->setRate(0.1, 0.3); // percent/s
    actuators[T3]->setRate(0.1, 0.3); // percent/s
    actuators[T4]->setRate(0.1, 0.3); // percent/s
}

// **********************************************************************
// Actuator Type Base
// **********************************************************************
ActuatorTypeBase::ActuatorTypeBase(ModelMap *pMapInit, bool debugFlagIn, double init_position)
{
    pTime     = NULL;
    pMap      = pMapInit;
    debugFlag = debugFlagIn;
    
    command    = 0.0;
    position   = init_position;
    lowerLimit = 0.0;
    upperLimit = 0.0;
    limitsSet  = false;
    inputMode  = external;
    
    pTable = NULL;
    
    terminal = new terminalType(debugFlagIn);
}

ActuatorTypeBase::~ActuatorTypeBase(void)
{
    delete terminal;
}

void ActuatorTypeBase::initialize(void)
{
    pTime = (Time*) pMap->getModel("Time");
    Base::initialize(pTime);
}

bool ActuatorTypeBase::update(void)
{
    updateDt(pTime);
    
    if (inputMode == keyboard) { updateKeyboard(); }
    if (inputMode == table)    { updateTable(); }
    
    updateDynamics();
    
    applyLimits();
    
    return true;
}

void ActuatorTypeBase::updateDynamics(void)
{
    position = command;
}

void ActuatorTypeBase::applyLimits(void)
{
    if (limitsSet)
    {
        if (position > upperLimit) { position = upperLimit; }
        if (position < lowerLimit) { position = lowerLimit; }
    }
}

void ActuatorTypeBase::updateKeyboard(void)
{
    terminal->update();
    double rate = terminal->getRate();
    command += rate*dt;
}

void ActuatorTypeBase::updateTable(void)
{
    if (pTable == NULL) return;

    if (debugFlag) { std::cout<<"Table:"; }
    for (int i=0; i<pTable->tableLength; i++)
    {
        if (debugFlag) { std::cout<<" "<<pTable->pTableTimes[i]<<" "<<pTable->pTableValues[i]; }
        if( pTime->getSimTime() >= pTable->pTableTimes[i] )
        {
            command = pTable->pTableValues[i];
        }
    }
    if (debugFlag) { std::cout<<std::endl; }
}

void ActuatorTypeBase::terminalType::update()
{
    if (pTerminal)
    {
        // Monitor input
        int input = pTerminal->monitorInput();
        if (input != '\0')
        {
            // Actuator rates
            if (input == upKey) // increase
            {
                actuatorRateRaw = baseRate;
                tempState = increasing;
            }
            else if (input == downKey) // decrease
            {
                actuatorRateRaw = -baseRate;
                tempState = decreasing;
            }
            else
            {
                actuatorRateRaw = 0;
                tempState = off;
            }
        }
        else
        {
            actuatorRateRaw = 0.0;
            tempState = off;
        }
        
        // Apply rates
        if (pDebounce != NULL)
        {
            // Update debounce time if state has changed
            if (prevActuatorState != actuatorState)
            {
                if (actuatorState == off) { pDebounce->setDelay(onDelay); }
                else { pDebounce->setDelay(offDelay); }
                prevActuatorState = actuatorState;
            }
            
            // Determine if debounce time has elapsed
            pDebounce->delayValue(&actuatorRate, actuatorRateRaw);
            
            // Apply state change
            if ( pDebounce->getChangeApplied() )
            {
                if (debugFlag) { printKeyPress(input); }
                actuatorState = tempState;
            }
        }
        else
        {
            actuatorRate = actuatorRateRaw;
        }
    }
}

void ActuatorTypeBase::terminalType::printKeyPress(int input)
{
    if (input == upArrow)    { printf("up arrow pressed\n"); }
    if (input == downArrow)  { printf("down arrow pressed\n"); }
    if (input == leftArrow)  { printf("left arrow pressed\n"); }
    if (input == rightArrow) { printf("right arrow pressed\n"); }
    if (input == wKey)       { printf("w pressed\n"); }
    if (input == sKey)       { printf("s pressed\n"); }
    if (input == aKey)       { printf("a pressed\n"); }
    if (input == dKey)       { printf("d pressed\n"); }
}

// **********************************************************************
// ServoMotor
// **********************************************************************
ServoMotor::ServoMotor(ModelMap *pMapInit, bool debugFlagIn, double init_position) : ActuatorTypeBase(pMapInit, debugFlagIn, init_position)
{
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    position = init_position;
    
    positionDynamics.setInitialValue(position);
    //pMap->addLogVar("PWM", &bodyForce[0], printSavePlot, 3);
}

void ServoMotor::updateDynamics(void)
{
    positionDynamics.setValue(command);
    position = positionDynamics.getFilterValue();
}

// **********************************************************************
// Brushless DC Motor
// **********************************************************************
BrushlessDCMotor::BrushlessDCMotor(ModelMap *pMapInit, bool debugFlagIn, double init_position) : ActuatorTypeBase(pMapInit, debugFlagIn, init_position)
{
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    position = init_position;
    //pMap->addLogVar("xForce", &bodyForce[0], printSavePlot, 3);
    
    pESC = new Delay(&counter, 1.0f/dynamicsInterval_init, 0.0);
}

BrushlessDCMotor::~BrushlessDCMotor(void)
{
    delete pESC;
}

void BrushlessDCMotor::updateDynamics(void)
{
    // Delay from command ESC to ESC creating command
    pESC->delayValue(&position, command);
    if (debugFlag)
    {
        std::cout<<"command: " << command;
        std::cout<<" position: "<< position << std::endl;
    }
}
