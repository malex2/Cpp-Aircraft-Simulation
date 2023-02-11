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
#include "arduino_class_models.hpp"

GenericModel::GenericModel()
{
    pMap  = NULL;
    
    debugFlag = false;
    
    time     = 0.0;
    prevTime = 0.0;
    counter  = 0.0;
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

void GenericModel::setDebugFlag(bool flag)
{
    debugFlag = flag;
}

GenericForceModel::GenericForceModel()
{
    util.setArray(LLForce, zero_init, 3);
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
}

GenericSensorModel::GenericSensorModel()
{
    pSerialIO = 0;
    perfectSensor = false;
    util.initArray(randomNoise, 0.0, 3);
}

double* GenericSensorModel::randomNoiseModel(double* maxNoise)
{
    util.initArray(randomNoise, 0.0, 3);
    for (int i=0; i<3; i++)
    {
        randomNoise[i] = randomNoiseModel( *(maxNoise+i) );
    }
    return &randomNoise[0];
}

double GenericSensorModel::randomNoiseModel(double maxNoise)
{
    // update random seed
    static unsigned int count = 0;
    srand(++count);
    
    int randInt = rand() % 100;
    double randToMax = maxNoise*randInt / 100.0;
    double randMinMax = randToMax*2.0 - maxNoise;
    return randMinMax;
}

void GenericSensorModel::setSerialIO(SimulationSerial* pIO)
{
    pSerialIO = pIO;
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
