//
//  general_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/12/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "initial_conditions.hpp"
#include "generic_model.hpp"
#include "time.hpp"

GenericModel::GenericModel()
{
    pMap  = NULL;
    
    debugFlag = false;
    
    time     = 0;
    prevTime = 0;
    counter  = 0;
}

void GenericModel::initialize(Time* pTime)
{
    time = pTime->getSimTime();
}

void GenericModel::updateDt(Time* pTime)
{
    if (pTime)
    {
        counter = pTime->getCount();
        
        time = pTime->getSimTime();
        dt = time - prevTime;
        prevTime = time;
    }
}

GenericForceModel::GenericForceModel()
{
    util.setArray(LLForce, zero_init, 3);
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
}

// Example class
/*
#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "time.hpp"
*/
ExampleModel::ExampleModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    
    pMap = pMapInit;
    
    //pMap->addLogVar("xForce", &bodyForce[0], printSavePlot, 3);
    
    debugFlag = debugFlagIn;
}

void ExampleModel::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    pTime   = (Time*)          pMap->getModel("Time");
}

bool ExampleModel::update(void)
{
    return true;
}
