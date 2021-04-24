//
//  Atmosphere.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/28/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"

#include "atmosphere_model.hpp"

AtmosphereModel::AtmosphereModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pRotate = NULL;
    pDyn    = NULL;
    pMap    = pMapInit;
    
    
    //pMap->addLogVar("Body Gravity X", &bodyGravity[0], savePlot, 2);
    //pMap->addLogVar("Body Gravity Y", &bodyGravity[1], savePlot, 2);
    //pMap->addLogVar("Body Gravity Z", &bodyGravity[2], savePlot, 2);
    //pMap->addLogVar("Grav", &gravity, savePlot, 2);
    //pMap->addLogVar("Wx  ", &bodyForce[0], savePlot, 2);
    //pMap->addLogVar("Wy  ", &bodyForce[1], savePlot, 2);
    //pMap->addLogVar("Wz  ", &bodyForce[2], savePlot, 2);
    //pMap->addLogVar("W   ", &LLForce[2], savePlot, 3);
    //pMap->addLogVar("Re  ", &Re, savePlot, 2);
    //pMap->addLogVar("Mach", &Mach, savePlot, 2);
    
    
    util.setArray(nedGravity, zero_init, 3);
    util.setArray(bodyGravity, zero_init, 3);
    util.setArray(nedForce, zero_init, 3);
    util.setUnitClassArray(velWindNED, zero_init, metersPerSecond, 3);
    util.setUnitClassArray(velWindBody, zero_init, metersPerSecond, 3);
    
    speedOfSound.convertUnit(metersPerSecond);
    Mach = 0;
    Re   = 0;
    gravity = 9.81;
    
    air[density]      = 1.225;  // kg/m^3
    air[pressure]     = 101325; // N/m^2 (Pa)
    air[dynPress]     = 0;      // N/m^2 (Pa)
    air[temp]         = 288.16; // K
    air[dynVisc]      = 0.0000181;
    air[specificHeat] = 1.40;     // J/(mol K)
    air[R]            = 8.314510; // J/(mol K)
    
    debugFlag = debugFlagIn;
    
}

void AtmosphereModel::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    
    updateAir();
}

bool AtmosphereModel::update(void)
{
    updateGravity();
    
    updateAir();
    
    updateWind();

    pRotate->bodyToLL(LLForce, bodyForce);
    
    if (debugFlag)
    {
        printf("Atmosphere Model Update:\n");
        util.print(nedForce, 3, "NED Gravity");
        util.print(bodyForce, 3, "Body Gravity");
    }
    return true;
}

void AtmosphereModel::updateGravity(void)
{
    double hCenter = pDyn->gethCenter().m();
    double mass = pDyn->getMass();
    gravity = GM/(hCenter*hCenter);
    
    nedGravity[2] = gravity;
    pRotate->NEDToBody(bodyGravity, nedGravity);
    
    nedForce[2] = mass*gravity;
    
    pRotate->NEDToBody(bodyForce, nedForce);
}

void AtmosphereModel::updateAir(void)
{
    // altitude and speed
    //double alt = pDyn->getPosBody()[2];
    double speed = pDyn->getSpeed().mps();
    
    // density
    air[density]      = 1.225;  // kg/m^3
    
    // pressure
    air[pressure]     = 101325; // N/m^2 (Pa)
    air[dynPress]     = 0.5 * air[density] * speed*speed;
    //temperature
    air[temp]         = 288.16; // K
    
    // dynamic viscocity
    air[dynVisc]      = 0.0000181;
    
    // Mach number
    speedOfSound.val = sqrt( air[specificHeat] * air[R] * air[temp] );
    Mach = speed / speedOfSound.mps();
    
    //Reynolds number
    Re = air[density] * speed * meanChord / air[dynVisc];
    
}

void AtmosphereModel::updateWind(void)
{
    velWindNED[0].val = 0;
    velWindNED[1].val = 0;
    velWindNED[2].val = 0;
    
    pRotate->NEDToBody(velWindBody, velWindNED);
}
