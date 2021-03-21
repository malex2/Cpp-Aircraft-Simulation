//
//  fs_pwmin.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_pwmin_hpp
#define fs_pwmin_hpp

#include "fs_common.hpp"

// Functions
void FsPwmIn_setup();
void FsPwmIn_performPwmIn();

// Getters
unsigned long* FsPwmIn_getPWM();
double*        FsPwmIn_getValues();

class PwmIn {
#ifdef SIMULATION
public:
    // Constructor
    PwmIn();
    
    void attach(int pinIn);
    
    unsigned long getPwm();
private:
    
    struct tableType
    {
        double *pTableValues;
        double *pTableTimes;
        int    tableLength;
    };
    tableType *pTable;
    
    // Variables
    unsigned int Pins[nChannels];
    double minValues[nChannels];
    double maxValues[nChannels];
    
    channelType channel;
    unsigned long pwm;
    double value;
    double minValue;
    double maxValue;
    bool foundChannel;
    
    // Tables
    static const int lengthTable = 4;
    
    double signalTimes[nChannels][lengthTable] = {
        {0, 5, 15, 20},
        {0, 5, 15, 20},
        {0, 5, 15, 20},
        {0, 5, 15, 20}
    };
    
    double signalValues[nChannels][lengthTable] = {
        {0, 100, 100, 60}, // Throttle (%), 0-100
        {0, 0 , 5 , -5}, // Aileron  (deg)
        {0, 0 , 0 , 0 }, // Pitch    (deg)
        {0, 0 , 0 , 0 }  // Yaw      (deg)
    };
#else
public:
    // Constructor
    PwmIn();
    
    // Functions
    void attach(int pinIn);
    
    // Instance Getters
    unsigned long getPwm();
    int getPin()    { return thisPin; }
    int getPinLoc() { return thisPinLoc; }
    static int getCurPin() { return pinArray[iPin]; }
    
    // Interrupts
    static void riseInterrupt();
    static void fallInterrupt();
    
    // Constants
    static const int maxPins = 6;
    static const int minPWM = 995;
    static const int maxPWM = 2005;
private:
    // Instance Info
    int thisPin;
    int thisPinLoc;
    
    // Pin Info
    static int iPin;  // Position of pin in pinArray
    static int nPins; // Number of interrupt pins
    static int pinArray[maxPins];     // Array of interrupt pins
    static volatile int tRise;        // Rise time for current interrupt
    static volatile int pwm[maxPins]; // Array of interrupt pwms
#endif
};

#endif /* fs_pwmin_hpp */
