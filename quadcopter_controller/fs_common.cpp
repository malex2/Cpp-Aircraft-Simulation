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

// Errors
double errorToVariance(double maxError)
{
    double std;
    double variance;
    
    // max error is 3 standard deviations
    std = maxError / 3.0;
    // variance is std^2
    variance = std*std;
    
    return variance;
}

double vectorMag(double* vec)
{
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void crossProduct(double *cross, double *a, double *b)
{
    *(cross+0) = a[1]*b[2] - a[2]*b[1];
    *(cross+1) = a[2]*b[0] - a[0]*b[2];
    *(cross+2) = a[0]*b[1] - a[1]*b[0];
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
template void display(double);
template void display(byte);

#ifdef SIMULATION
void FsCommon_setSimulationModels(ModelMap* pMap)
{
    if (pMap) { pTime = (Time*) pMap->getModel("Time"); }
}
#endif
