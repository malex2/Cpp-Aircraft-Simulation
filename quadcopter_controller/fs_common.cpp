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
#endif

// Classes
#ifdef SIMULATION
    class Time* pTime = 0;
#endif

// Time
double getTime()
{
#ifdef SIMULATION
    if (pTime) { return pTime->getSimTime(); }
    else { return 0.0; }
#else
    return micros() / 1000000.0;
#endif
}

// LED
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

// Printing
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
template void display(unsigned int);
template void display(float);
template void display(double);
template void display(unsigned long);

void FsCommon_setSimulationModels(ModelMap* pMap)
{
#ifdef SIMULATION
    if (pMap) { pTime = (Time*) pMap->getModel("Time"); }
#endif
}
