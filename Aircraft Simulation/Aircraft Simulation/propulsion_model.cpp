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

#include "propulsion_model.hpp"

PropulsionModel::PropulsionModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    
    pMap = pMapInit;
    
    pMap->addLogVar("throttle", &throttle, printSavePlot, 3);
    pMap->addLogVar("thrust", &bodyForce[0], savePlot, 2);
    pMap->addLogVar("propLLX", &LLForce[0], savePlot, 2);
    
    throttle = actuators_init[3];
    
    debugFlag = debugFlagIn;
}

void PropulsionModel::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
}

bool PropulsionModel::update(void)
{
    bodyForce[0] = throttle*maxThrust;
    bodyForce[1] = 0;
    bodyForce[2] = 0;
    
    bodyMoment[0] = 0;
    bodyMoment[1] = 0;
    bodyMoment[2] = 0;
    
    pRotate->bodyToLL(LLForce, bodyForce);
    
    if (debugFlag)
    {
        printf("Propulsion Model Update:\n");
        util.print(bodyForce, 3, "bodyForce");
        util.print(LLForce, 3, "LLForce");
    }
    return true;
}
