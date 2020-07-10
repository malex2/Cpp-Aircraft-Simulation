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

#include "actuator_model.hpp"

ActuatorModel::ActuatorModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pProp   = NULL;
    pMap     = pMapInit;
    
    pMap->addLogVar("de  ", &actuators[de], printSavePlot, 3);
    pMap->addLogVar("da   ", &actuators[da], printSavePlot, 3);
    pMap->addLogVar("dr   ", &actuators[dr], printSavePlot, 3);
    pMap->addLogVar("throttle", &actuators[dT], savePlot, 2);
    
    pMap->addLogVar("elevator rate", &actuatorRates[de], savePlot, 2);
    pMap->addLogVar("aileron rate", &actuatorRates[da], savePlot, 2);
    pMap->addLogVar("rudder rate", &actuatorRates[dr], savePlot, 2);
    pMap->addLogVar("throttle rate", &actuatorRates[dT], savePlot, 2);
    
    pMap->addLogVar("delevator", &dActuators[de], savePlot, 2);
    pMap->addLogVar("daileron", &dActuators[da], savePlot, 2);
    pMap->addLogVar("drudder", &dActuators[dr], savePlot, 2);
    pMap->addLogVar("dthrottle", &dActuators[dT], savePlot, 2);
    
    util.setArray(actuators, actuators_init, nActuators);

    util.initArray(actuatorRatesRaw, 0.0f, nActuators);
    util.initArray(actuatorRates, 0.0f, nActuators);
    
    inputMode = inputMode_init;
    
    if (inputMode == keyboard)
    {
        // Keyboard keys
        actuatorUpKey[de]   = sKey; // check
        actuatorDownKey[de] = wKey; // check
    
        actuatorUpKey[da]   = rightArrow; // check
        actuatorDownKey[da] = leftArrow;  // check
    
        actuatorUpKey[dr]   = dKey; // check
        actuatorDownKey[dr] = aKey; // check
    
        actuatorUpKey[dT]   = upArrow;
        actuatorDownKey[dT] = downArrow;
    
        // Rates
        baseRate[de] = 1.0; // deg/s
        baseRate[da] = 1.0; // deg/s
        baseRate[dr] = 1.0; // deg/s
        baseRate[dT] = 0.1; // percent/s
        
        // on/off delays
        onDelay[de] = 0;
        onDelay[da] = 0;
        onDelay[dr] = 0;
        onDelay[dT] = 0;
    
        offDelay[de] = 0.2;
        offDelay[da] = 0.2;
        offDelay[dr] = 0.2;
        offDelay[dT] = 0.6;

        // Set up terminal monitering
        pTerminal = new MonitorMacInput;
        
        // Set up debouncers
        for (int iAct = 0; iAct < nActuators; iAct++)
        {
            pDebounce[iAct] = new Delay(&counter, 1.0f/clock_dt, onDelay[iAct]);
            prevActuatorState[iAct] = off;
            actuatorState[iAct]     = off;
            tempState[iAct]         = off;
        }
    }
    
    dt = dynamicsInterval_init;
    
    debugFlag = debugFlagIn;
}

ActuatorModel::~ActuatorModel(void)
{
    if (inputMode == keyboard)
    {
        delete pTerminal;
        for (int iAct = 0; iAct < nActuators; iAct++)
        {
            delete pDebounce[iAct];
        }
    }
}

void ActuatorModel::initialize(void)
{
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pTime   = (Time*)            pMap->getModel("Time");
    pProp   = (PropulsionModel*) pMap->getModel("PropulsionModel");
    
    Base::initialize(pTime);
    
    pProp->setThrottle(actuators[dT]);
}

bool ActuatorModel::update(void)
{
    updateDt(pTime);
    
    // Collect inputs
    if (inputMode == keyboard) { updateKeyboard(); }
    
    if (inputMode == serial) { updateSerial(); }

    // Actuator dynamics
    actuatorDynamics();
    
    pProp->setThrottle(actuators[dT]);
    
    return true;
}

void ActuatorModel::updateKeyboard(void)
{
    if (pTerminal)
    {
        // Monitor input
        int input = pTerminal->monitorInput();
        if (input != '\0')
        {
            // Actuator rates
            for (int iAct = 0; iAct < nActuators; iAct++)
            {
                if (input == actuatorUpKey[iAct]) // increase
                {
                    actuatorRatesRaw[iAct] = baseRate[iAct];
                    tempState[iAct] = increasing;
                }
                
                else if (input == actuatorDownKey[iAct]) // decrease
                {
                    actuatorRatesRaw[iAct] = -baseRate[iAct];
                    tempState[iAct] = decreasing;
                }
                
                else
                {
                    actuatorRatesRaw[iAct] = 0;
                    tempState[iAct] = off;
                }
            }
        }
        else
        {
            util.initArray(actuatorRatesRaw, 0.0f, nActuators);
            util.initArray((int*)tempState, (int)off, nActuators);
        }
        
        // Apply rates
        for (int iAct = 0; iAct < nActuators; iAct++ )
        {
            // Update debounce time if state has changed
            if (prevActuatorState[iAct] != actuatorState[iAct])
            {
                if (actuatorState[iAct] == off) {
                    pDebounce[iAct]->setDelay(onDelay[iAct]);
                }
                else {
                    pDebounce[iAct]->setDelay(offDelay[iAct]);
                }
                prevActuatorState[iAct] = actuatorState[iAct];
            }
            
            // Determine if debounce time has elapsed
            pDebounce[iAct]->delayValue(&actuatorRates[iAct], actuatorRatesRaw[iAct]);
            
            // Apply state change
            if ( pDebounce[iAct]->getChangeApplied() )
            {
                if (debugFlag) { printKeyPress(input); }
                actuatorState[iAct] = tempState[iAct];
            }
            
            // Apply rate to actuators
            dActuators[iAct] = actuatorRates[iAct]*dt;
            actuators[iAct] += dActuators[iAct];
        }
    }
    
    if (debugFlag)
    {
        util.print(actuatorRates, nActuators, "actuator rates:");
        util.print(actuators, nActuators, "actuator:");
    }
}

void ActuatorModel::updateSerial(void)
{
    
}

void ActuatorModel::actuatorDynamics(void)
{
    
}

void ActuatorModel::printKeyPress(int input)
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
