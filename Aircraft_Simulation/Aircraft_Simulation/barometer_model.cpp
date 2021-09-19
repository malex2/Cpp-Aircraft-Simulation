//
//  barometer_model.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 8/27/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "barometer_model.hpp"
#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "atmosphere_model.hpp"
#include "time.hpp"

BarometerModelBase::BarometerModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn  = NULL;
    pAtmo = NULL;
    pTime = NULL;
    
    pMap = pMapInit;
    
    //pMap->addLogVar("baro model pressure mbar", &pressure, savePlot, 2);
    //pMap->addLogVar("baro model temp C", &temperature, savePlot, 2);
    //pMap->addLogVar("baro model pressure reading", &pressureSensor, savePlot, 2);
    //pMap->addLogVar("baro model temperature reading", &temperatureSensor, savePlot, 2);
    //pMap->addLogVar("baro model pressure noise", &pressureError, savePlot, 2);
    //pMap->addLogVar("baro model temperature noise", &temperatureError, savePlot, 2);
    //pMap->addLogVar("pressure check", &pressureCheck, savePlot, 2);
    
    pressure = 0.0;
    temperature = 0.0;
    perfectSensor = false;
    pressureSensor = 0.0;
    temperatureSensor = 0.0;
    debugFlag = debugFlagIn;
    
    pressureNoiseMax = 0.0;
    pressureError = 0.0;
    
    temperatureNoiseMax = 0.0;
    temperatureError = 0.0;
}

void BarometerModelBase::initialize()
{
    pDyn  = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pAtmo = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pTime = (Time*)            pMap->getModel("Time");
    
    pressureNoise.setNoise( pressureNoiseMax );
    temperatureNoise.setNoise( temperatureNoiseMax );
}

bool BarometerModelBase::update()
{
    // Update enviroment
    pressure = pAtmo->getAir()[AtmosphereModel::pressure]*0.01;     // mbar
    temperature = pAtmo->getAir()[AtmosphereModel::temp] - 273.15; // degree C
    
    // Update raw pressure and temperature
    updateTemperature();
    updatePressure();
    
    return true;
}

void BarometerModelBase::updatePressure()
{
    pressureError = pressureNoise.getNoise();
    pressureSensor = pressure + pressureError;
}

void BarometerModelBase::updateTemperature()
{
    temperatureError = temperatureNoise.getNoise();
    temperatureSensor = temperature + temperatureError;
}

bmp180::bmp180(ModelMap *pMapInit, bool debugFlagIn) : BarometerModelBase(pMapInit, debugFlagIn)
{
    pMap = pMapInit;
    
    //pMap->addLogVar("baro model pu", &pu, savePlot, 2);
    //pMap->addLogVar("baro model tu", &tu, savePlot, 2);
    
    double c3, c4, b1;
    
    calData[AC1] = 8480;
    calData[AC2] = -1032;
    calData[AC3] = -14609;
    calData[AC4] = 33173;
    calData[AC5] = 25286;
    calData[AC6] = 18793;
    calData[VB1] = 6516;
    calData[VB2] = 34;
    calData[MB]  = -32768;
    calData[MC]  = -11786;
    calData[MD]  = 2646;
    
    temperatureNoiseMax = 2.0;
    pressureNoise0   = 0.06;
    pressureNoise1   = 0.05;
    pressureNoise2   = 0.04;
    pressureNoise3   = 0.03;
    pressureNoiseMax = pressureNoise0;
    
    temperatureDelay = 4.5/1000.0;
    pressureDelay0   = 4.5/1000.0;
    pressureDelay1   = 7.5/1000.0;
    pressureDelay2   = 13.5/1000.0;
    pressureDelay3   = 25.5/1000.0;
    pressureDelay    = pressureDelay0;
    
    tempReading.setDelay(temperatureDelay);
    presReading.setDelay(pressureDelay);
    lastRequest = NOREQUEST;
    
    c3 = 160.0 * pow(2, -15) * calData[AC3];
    c4 = pow(10, -3) * pow(2, -15) * calData[AC4];
    b1 = pow(160, 2) * pow(2, -30) * calData[VB1];
    c5 = (pow(2, -15) / 160) * calData[AC5];
    c6 = calData[AC6];
    mc = (pow(2, 11) / pow(160, 2)) * calData[MC];
    md = calData[MD] / 160.0;
    x0 = calData[AC1];
    x1 = 160.0 * pow(2, -13) * calData[AC2];
    x2 = pow(160, 2) * pow(2, -25) * calData[VB2];
    y0 = c4 * pow(2, 15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2, -20);
    p2 = 3038.0 * 100.0 * pow(2, -36);
}

bool bmp180::update()
{
    double time;
    
    Base::update();
    
    if (pTime) { time = pTime->getSimTime(); }
    else { time = 0.0; }
    
    temperatureSensor = tempReading.update(time, tu);
    pressureSensor = presReading.update(time, pu);
    
    return true;
}

void bmp180::updateTemperature()
{
    // tu = (data1 * 256.0) + data2;
    // a = c5 * (tu - c6);
    // baroData.temperature = a + (mc / (a + md));
    //
    // 1) Solve for a
    //   temp = a + mc/(a+md)
    //   temp*(a+md) = a*(a+md) + mc
    //   temp*a + temp*md = a^2 + a*md + mc
    //   0 = a^2 + a*(md-temp) + mc - temp*md
    //
    // 2) Solve for tu
    //    a = c5 * (tu - c6)
    //    a/c5 = tu - c6
    //    a/c5 + c6 = tu
    
    double solnReal[2];
    double solnImag[2];
    bool realSolution;
    double a;
    
    tempReading.setDelay(temperatureDelay);
    
    if (!perfectSensor) { temperature = temperatureNoise + temperature; }
    temperatureError = temperatureNoise.getNoise();
    
    // Solve for a
    realSolution = util.solveQuadratic(1.0, md-temperature, mc-temperature*md, solnReal, solnImag);
    if (!realSolution) { std::cout << "Warning - bmp180::updateTemperaute, imaginary solition" << std::endl; }
    
    if (solnReal[0] > 0) { a = solnReal[0]; }
    else { a = solnReal[1]; }
    
    // Solve for tu
    tu = c6 + a/c5;
}

void bmp180::updatePressure()
{
    //pu = (data1 * 256.0) + data2 + (data3 / 256.0);
    //s = baroData.temperature - 25.0;
    //x = (x2 * pow(s, 2)) + (x1 * s) + x0;
    //y = (y2 * pow(s, 2)) + (y1 * s) + y0;
    //z = (pu - x) / y;
    //baroData.pressure = (p2 * pow(z, 2)) + (p1 * z) + p0;
    //
    // 1) Solve for x
    //
    // 2) Solve for y
    //
    // 3) Solve for z
    //    p = p2*z^2 + p1*z + p0
    //    0 = p2*z^2 + p1*z + p0-p
    // 4) Solve for pu
    //    z = (pu - x) / y
    //    z*y = pu - x
    //    z*y + x = pu
    
    double solnReal[2];
    double solnImag[2];
    bool realSolution;
    double a;
    double lastTemperature;
    double x;
    double y;
    double z;
    double s;
    
    presReading.setDelay(pressureDelay);
    
    if (!perfectSensor) { pressure = pressureNoise + pressure; }
    pressureError = pressureNoise.getNoise();
    
    // Use last temperature measurement
    if (temperatureSensor != 0.0) {
        a = c5 * (temperatureSensor - c6);
        lastTemperature = a + (mc / (a + md));
    }
    else {
        lastTemperature = temperature;
    }
    
    s = lastTemperature - 25.0;
    x = x2*s*s + x1*s + x0;
    y = y2*s*s + y1*s + y0;
    
    // Solve for z
    realSolution = util.solveQuadratic(p2, p1, p0-pressure, solnReal, solnImag);
    if (!realSolution) { std::cout << "Warning - bmp180::updatePressure, imaginary solition" << std::endl; }
    if (solnReal[0] > 0) { z = solnReal[0]; }
    else { z = solnReal[1]; }
    
    // Solve for pu
    pu = x + y*z;
}

void bmp180::setPressureNoise(int noiseInt)
{
    if      (noiseInt == 0) { pressureNoise.setNoise( pressureNoise0 ); pressureDelay = pressureDelay0; }
    else if (noiseInt == 1) { pressureNoise.setNoise( pressureNoise1 ); pressureDelay = pressureDelay1; }
    else if (noiseInt == 2) { pressureNoise.setNoise( pressureNoise2 ); pressureDelay = pressureDelay2; }
    else if (noiseInt == 3) { pressureNoise.setNoise( pressureNoise3 ); pressureDelay = pressureDelay3; }
    else { std::cout << "bmp180::setPressureNoise - Invalid pressure noise" << std::endl;}
}
