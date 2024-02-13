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
    //pMap->addLogVar("gravity_roll", &gravity_roll_deg, savePlot, 2);
    //pMap->addLogVar("gravity_pitch", &gravity_pitch_deg, savePlot, 2);
    //pMap->addLogVar("Grav", &gravity, savePlot, 2);
    //pMap->addLogVar("Wx  ", &bodyForce[0], savePlot, 2);
    //pMap->addLogVar("Wy  ", &bodyForce[1], savePlot, 2);
    //pMap->addLogVar("Wz  ", &bodyForce[2], savePlot, 2);
    //pMap->addLogVar("W   ", &LLForce[2], savePlot, 2);
    //pMap->addLogVar("desnity", &air[density], printSavePlot, 3);
    //pMap->addLogVar("pressure", &air[pressure], savePlot, 2);
    //pMap->addLogVar("temperature", &air[temp], savePlot, 2);
    //pMap->addLogVar("pressure (mb)", &pressureMB, savePlot, 2);
    //pMap->addLogVar("temperature (C)", &tempC, savePlot, 2);
    //pMap->addLogVar("dynamic pressure", &air[dynPress], printSavePlot, 3);
    //pMap->addLogVar("layer", &layer, printSavePlot, 3);
    //pMap->addLogVar("Re  ", &Re, savePlot, 2);
    //pMap->addLogVar("Mach", &Mach, savePlot, 2);
    
    util.setArray(nedGravity, zero_init, 3);
    util.setArray(bodyGravity, zero_init, 3);
    util.setArray(nedForce, zero_init, 3);
    util.setArray(velWindNED, zero_init, 3);
    util.setArray(velWindBody, zero_init, 3);
    
    speedOfSound.convertUnit(metersPerSecond);
    Mach = 0.0;
    Re   = 0.0;
    gravity = 9.81;
    pressureMB = 0.0;
    tempC = 0.0;
    layer = (double) TROPOSPHERE;
    
    air[density]      = 1.225;  // kg/m^3
    air[pressure]     = 101325.0; // N/m^2 (Pa)
    air[dynPress]     = 0.0;      // N/m^2 (Pa)
    air[dynVisc]      = 0.0000185;
    air[specificHeat] = 1.40;     // kJ/(kg K)
    air[R]            = 287.0; // J/(kg K)  [=8.31451 J/(mol K)]
    air[temp]         = 288.16; // K
    
    ALTITUDE[TROPOSPHERE]    = 0.0;
    TEMPERATURE[TROPOSPHERE] = 288.16;
    PRESSURE[TROPOSPHERE]    = 101325;
    DENSITY[TROPOSPHERE]     = 1.225;
    DTDH[TROPOSPHERE]        = -6.5e-3;
    GRADIENT[TROPOSPHERE]    = SLOPE;
    
    ALTITUDE[STRATOSPHERE1]    = 11000.0;
    TEMPERATURE[STRATOSPHERE1] = 216.66;
    PRESSURE[STRATOSPHERE1]    = gradientPressure(PRESSURE[TROPOSPHERE], TEMPERATURE[TROPOSPHERE], TEMPERATURE[STRATOSPHERE1], DTDH[TROPOSPHERE]);
    DENSITY[STRATOSPHERE1]     = gradientDensity(DENSITY[TROPOSPHERE], TEMPERATURE[TROPOSPHERE], TEMPERATURE[STRATOSPHERE1], DTDH[TROPOSPHERE]);
    DTDH[STRATOSPHERE1]        = 0.0;
    GRADIENT[STRATOSPHERE1]    = CONSTANT;
    
    ALTITUDE[STRATOSPHERE2]    = 25000.0;
    TEMPERATURE[STRATOSPHERE2] = 216.66;
    PRESSURE[STRATOSPHERE2]    = constantPressure(PRESSURE[STRATOSPHERE1], ALTITUDE[STRATOSPHERE1], ALTITUDE[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE2]);
    DENSITY[STRATOSPHERE2]     = constantDensity(DENSITY[STRATOSPHERE1], ALTITUDE[STRATOSPHERE1], ALTITUDE[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE2]);
    DTDH[STRATOSPHERE2]        = 3.0e-3;
    GRADIENT[STRATOSPHERE2]    = SLOPE;
    
    ALTITUDE[STRATOSPHERE3]    = 47000.0;
    TEMPERATURE[STRATOSPHERE3] = 282.66;
    PRESSURE[STRATOSPHERE3]    = gradientPressure(PRESSURE[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE3], DTDH[STRATOSPHERE2]);
    DENSITY[STRATOSPHERE3]     = gradientDensity(DENSITY[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE2], TEMPERATURE[STRATOSPHERE3], DTDH[STRATOSPHERE2]);
    DTDH[STRATOSPHERE3]        = 0.0;
    GRADIENT[STRATOSPHERE3]    = CONSTANT;
    
    ALTITUDE[MESOSPHERE1]    = 53000.0;
    TEMPERATURE[MESOSPHERE1] = 282.66;
    PRESSURE[MESOSPHERE1]    = constantPressure(PRESSURE[STRATOSPHERE3], ALTITUDE[STRATOSPHERE3], ALTITUDE[MESOSPHERE1], TEMPERATURE[MESOSPHERE1]);
    DENSITY[MESOSPHERE1]     = constantDensity(DENSITY[STRATOSPHERE3], ALTITUDE[STRATOSPHERE3], ALTITUDE[MESOSPHERE1], TEMPERATURE[MESOSPHERE1]);
    DTDH[MESOSPHERE1]        = -4.5e-3;
    GRADIENT[MESOSPHERE1]    = SLOPE;
    
    ALTITUDE[MESOSPHERE2]    = 79000.0;
    TEMPERATURE[MESOSPHERE2] = 165.66;
    PRESSURE[MESOSPHERE2]    = gradientPressure(PRESSURE[MESOSPHERE1], TEMPERATURE[MESOSPHERE1], TEMPERATURE[MESOSPHERE2], DTDH[MESOSPHERE1]);
    DENSITY[MESOSPHERE2]     = gradientDensity(DENSITY[MESOSPHERE1], TEMPERATURE[MESOSPHERE1], TEMPERATURE[MESOSPHERE2], DTDH[MESOSPHERE1]);
    DTDH[MESOSPHERE2]        = 0.0;
    GRADIENT[MESOSPHERE2]    = CONSTANT;
    
    ALTITUDE[THERMOSPHERE1]    = 90000.0;
    TEMPERATURE[THERMOSPHERE1] = 165.66;
    PRESSURE[THERMOSPHERE1]    = constantPressure(PRESSURE[MESOSPHERE2], ALTITUDE[MESOSPHERE2], ALTITUDE[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE1]);
    DENSITY[THERMOSPHERE1]     = constantDensity(DENSITY[MESOSPHERE2], ALTITUDE[MESOSPHERE2], ALTITUDE[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE1]);
    DTDH[THERMOSPHERE1]        = 4.0e-3;
    GRADIENT[THERMOSPHERE1]    = SLOPE;
    
    ALTITUDE[THERMOSPHERE2]    = 105000.0;
    TEMPERATURE[THERMOSPHERE2] = 225.66;
    PRESSURE[THERMOSPHERE2]    = gradientPressure(PRESSURE[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE2], DTDH[THERMOSPHERE1]);
    DENSITY[THERMOSPHERE2]     = gradientDensity(DENSITY[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE1], TEMPERATURE[THERMOSPHERE2], DTDH[THERMOSPHERE1]);
    DTDH[THERMOSPHERE2]        = -4.5e-3;
    GRADIENT[THERMOSPHERE2]    = CONSTANT;
    
    gravity_roll  = 0.0;
    gravity_pitch = 0.0;
    
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
    double hCenter = pDyn->gethCenter();
    double mass = pDyn->getMass();
    gravity = GM/(hCenter*hCenter);
    
    nedGravity[2] = gravity;
    pRotate->NEDToBody(bodyGravity, nedGravity);
    util.setArray(gravityEuler, pDyn->getEulerAngles(), 3);
    
    nedForce[2] = mass*gravity;
    pRotate->NEDToBody(bodyForce, nedForce);
    
    gravity_roll  = atan2(bodyGravity[1], bodyGravity[2]);
    gravity_pitch = -asin(bodyGravity[0]/gravity);
    
    // print variables
    gravity_roll_deg  = gravity_roll / util.deg2rad;
    gravity_pitch_deg = gravity_pitch/ util.deg2rad;
}

void AtmosphereModel::updateAir(void)
{
    // altitude and speed
    double alt = pDyn->getPosLLH()[2];
    double speed = pDyn->getSpeed();
    
    // Update atmospheric layer
    updateLayerInfo(alt);
    
    layer = layerInfo.layer;
    
    if (layerInfo.gradient == CONSTANT)
    {
        air[temp]     = layerInfo.baseTemperature;
        air[pressure] = constantPressure(layerInfo.basePressure, layerInfo.baseAltitude, alt, air[temp]);
        air[density]  = constantDensity(layerInfo.baseDensity, layerInfo.baseAltitude, alt, air[temp]);
    }
    else
    {
        air[temp]     = gradientTemperature(layerInfo.baseTemperature, layerInfo.baseAltitude, alt, layerInfo.dTdH);
        air[pressure] = gradientPressure(layerInfo.basePressure, layerInfo.baseTemperature, air[temp], layerInfo.dTdH);
        air[density]  = gradientDensity(layerInfo.baseDensity, layerInfo.baseTemperature, air[temp], layerInfo.dTdH);
    }
    
    air[dynVisc]      = 2.791e-7 * pow(air[temp], 0.7355);
    air[dynPress]     = 0.5 * air[density] * speed*speed;
    
    speedOfSound.val = sqrt( air[specificHeat] * air[R] * air[temp] );
    Mach = speed / speedOfSound.mps();
    Re   = air[density] * speed * meanChord / air[dynVisc];
    
    // print variables
    pressureMB = air[pressure]*0.01;
    tempC = air[temp] - 273.15;
}

void AtmosphereModel::updateWind(void)
{
    velWindNED[0] = 0.0;
    velWindNED[1] = 0.0;
    velWindNED[2] = 0.0;
    
    pRotate->NEDToBody(velWindBody, velWindNED);
}

double AtmosphereModel::constantPressure(double basePressure, double baseAltitude, double altitude, double temperature)
{
    double dH = altitude - baseAltitude;
    double exponent = -gravitySL/(air[R]*temperature)*dH;
    
    return basePressure*exp(exponent);
}

double AtmosphereModel::constantDensity(double baseDensity, double baseAltitude, double altitude, double temperature)
{
    double dH = altitude - baseAltitude;
    double exponent = -gravitySL/(air[R]*temperature)*dH;
    
    return baseDensity*exp(exponent);
}

double AtmosphereModel::gradientPressure(double basePressure, double baseTemperature, double temperature, double dTdH)
{
    double tempRatio = temperature/baseTemperature;
    double exponent = -gravitySL/(dTdH*air[R]);
    
    return basePressure*pow(tempRatio, exponent);
}

double AtmosphereModel::gradientDensity(double baseDensity, double baseTemperature, double temperature, double dTdH)
{
    double tempRatio = temperature/baseTemperature;
    double exponent = -(gravitySL/(dTdH*air[R]) + 1.0);
    
    return baseDensity*pow(tempRatio, exponent);
}

double AtmosphereModel::gradientTemperature(double baseTemperature, double baseAltitude, double altitude, double dTdH)
{
    return baseTemperature + dTdH*(altitude - baseAltitude);
}

void AtmosphereModel::updateLayerInfo(double altitude)
{
    static atmosphereType previousLayer = layerInfo.layer;
    
    for (int i=0; i<nATMOSPHERELAYERS; i++)
    {
        atmosphereType tempLayer = (atmosphereType) i;
        if (altitude >= ALTITUDE[tempLayer])
        {
            layerInfo.layer = tempLayer;
        }
    }
    
    if (previousLayer != layerInfo.layer)
    {
        std::cout << "AtmosphereModel::updateLayerInfo - Entering new layer " << layerInfo.layer << std::endl;
        previousLayer = layerInfo.layer;
    }
    
    layerInfo.gradient         = GRADIENT[layerInfo.layer];
    layerInfo.baseAltitude     = ALTITUDE[layerInfo.layer];
    layerInfo.basePressure     = PRESSURE[layerInfo.layer];
    layerInfo.baseDensity      = DENSITY[layerInfo.layer];
    layerInfo.baseTemperature  = TEMPERATURE[layerInfo.layer];
    layerInfo.dTdH             = DTDH[layerInfo.layer];
}
