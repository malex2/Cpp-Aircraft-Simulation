//
//  mass_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "mass_model.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "time.hpp"

MassModelBase::MassModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    
    pMap = pMapInit;
    
    //pMap->addLogVar("xForce", &bodyForce[0], printSavePlot, 3);
    
    debugFlag = debugFlagIn;
}

void MassModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    pTime   = (Time*)          pMap->getModel("Time");
}

bool MassModelBase::update(void)
{
    return true;
}

