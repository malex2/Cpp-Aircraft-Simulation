//
//  fs_common.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 2/26/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_common.hpp"

#ifdef SIMULATION
    #include "time.hpp"
    //#include "actuator_model.hpp"
    //#include "rotate_frame.hpp"
    //#include "dynamics_model.hpp"
#else
    #include "Servo.h"
    #include "Wire.h" // This library allows you to communicate with I2C devices.
    #define EI_ARDUINO_INTERRUPTED_PIN
    #include <EnableInterrupt.h>
#endif

// Classes
class Time* pTime = 0;

// Functions
double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = (valueMax-valueMin) / static_cast<double>(pwmMax-pwmMin);
    return slope*(pwm-pwmMin) + valueMin;
}

unsigned long mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, unsigned long valueMin, unsigned long valueMax)
{
    // y - y1 = m * (x - x1)
    double slope = 1.0*(valueMax-valueMin) / (pwmMax-pwmMin);
    return slope*(pwm-pwmMin) + valueMin;
}

unsigned long mapToPwm(double value, double valueMin, double valueMax, unsigned long pwmMin, unsigned long pwmMax)
{
    // y - y1 = m * (x - x1)
    double slope = (pwmMax-pwmMin) / (valueMax-valueMin);
    return slope*(value-valueMin) + pwmMin;
}

unsigned long limit(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax)
{
    if (pwm < pwmMin)
    {
        display("Warning (limit): pwm below limit ");
        display(pwm);
        display("\n");
        return pwmMin;
    }
    if (pwm > pwmMax)
    {
        display("Warning (limit): pwm above limit ");
        display(pwm);
        display("\n");
        return pwmMax;
    }
    return pwm;
}


#ifndef SIMULATION
const unsigned int LEDPIN = LED_BUILTIN;
#endif

void LEDon()
{
#ifndef SIMULATION
    digitalWrite(LEDPIN, HIGH);
#endif
}
void LEDoff()
{
#ifndef SIMULATION
    digitalWrite(LEDPIN, LOW);
#endif
}


double getTime()
{
#ifdef SIMULATION
    if (pTime) { return pTime->getSimTime(); }
    else { return 0.0; }
#else
    return micros() / 1000000.0;
#endif
}

template<typename TempType>
void display(TempType val)
{
#ifdef SIMULATION
    std::cout << val;
#else
    Serial.print(val);
#endif
}

template void display(const char*);
template void display(String);
template void display(int);
template void display(float);
template void display(double);
template void display(unsigned long);

void FsCommon_setSimulationModels(ModelMap* pMap)
{
    if (pMap) { pTime = (Time*) pMap->getModel("Time"); }
}
