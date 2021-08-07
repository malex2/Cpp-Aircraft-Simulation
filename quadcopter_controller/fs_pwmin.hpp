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
int* FsPwmIn_getPWM();

class PwmIn {
#ifdef SIMULATION
public:
    // Constructor
    PwmIn();
    
    void attach(int pinIn);
    
    int getPwm();
private:
    // Functions
    int mapToPwm(double value, double valueMin, double valueMax, int pwmMin, int pwmMax);
    
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
    unsigned int pwm;
    double value;
    double minValue;
    double maxValue;
    bool foundChannel;
    
    // Tables
    static const int lengthTable = 5;
    
    double signalTimes[nChannels][lengthTable] = {
        {0.0, 5.0, 15.0, 25.0, 30.0},
        {0.0, 5.0, 15.0, 25.0, 30.0},
        {0.0, 5.0, 15.0, 25.0, 30.0},
        {0.0, 5.0, 15.0, 25.0, 30.0}
    };
    
    double signalValues[nChannels][lengthTable] = {
        {0.0, 70.0, 20.0 , 70.0 , 39.0}, // Throttle (%), 0-100
        {0.0, 0.0 , 0.0  , 0.0  , 3.0 }, // Aileron  (deg)
        {0.0, 0.0 , 0.0  , 0.0  , -3.0 }, // Pitch    (deg)
        {0.0, -1.0 , 1.0  , 0.0  , 0.0 }  // YawRate  (deg/s)
    };
#else
public:
    // Constructor
    PwmIn();
    
    // Functions
    void attach(int pinIn);
    
    // Instance Getters
    int getPwm();
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
