//
//  fs_pwmin.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_pwmin.hpp"

# ifndef SIMULATION
    #define EI_ARDUINO_INTERRUPTED_PIN
    #include <EnableInterrupt.h>
#endif

PwmIn throttleChannel;
PwmIn rollChannel;
PwmIn yawChannel;
PwmIn pitchChannel;

int pwmIn[nChannels];

void FsPwmIn_setup()
{
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        pwmIn[iCh]    = PWMMIN;
    }
    
    throttleChannel.attach(THROTTLEPIN);
    rollChannel.attach(ROLLPIN);
    pitchChannel.attach(PITCHPIN);
    yawChannel.attach(YAWPIN);
}

void FsPwmIn_performPwmIn()
{
    // Get Desired Aittutde
    pwmIn[THROTTLE] = throttleChannel.getPwm();
    pwmIn[ROLL]     = rollChannel.getPwm();
    pwmIn[PITCH]    = pitchChannel.getPwm();
    pwmIn[YAW]      = yawChannel.getPwm();
}

int* FsPwmIn_getPWM()    { return pwmIn; }

#ifdef SIMULATION
PwmIn::PwmIn()
{
    // Initialize Pins
    Pins[THROTTLE] = THROTTLEPIN; // CH3
    Pins[ROLL]     = ROLLPIN;     // CH4
    Pins[PITCH]    = PITCHPIN;    // CH5
    Pins[YAW]      = YAWPIN;      // CH6
    
    // Initialize Ranges
    minValues[THROTTLE] = 0.0;
    minValues[ROLL]     = -MAXROLL;
    minValues[PITCH]    = -MAXPITCH;
    minValues[YAW]      = -MAXYAWRATE;
    
    maxValues[THROTTLE] = 100.0;
    maxValues[ROLL]     = MAXROLL;
    maxValues[PITCH]    = MAXPITCH;
    maxValues[YAW]      = MAXYAWRATE;
    
    // Initialize Variables
    channel  = THROTTLE;
    pwm      = PWMMIN;
    minValue = minValues[channel];
    maxValue = maxValues[channel];
    value    = minValue;
    pTable   = NULL;
    foundChannel = false;
    
    // Table To Radians
    for (int iCh = ROLL; iCh != nChannels; iCh++)
    {
        for (int col = 0; col < lengthTable; col++)
        {
            signalValues[iCh][col] *= degree2radian;
        }
    }
}

void PwmIn::attach(int pinIn)
{
    // Find Channel
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        if (!foundChannel && pinIn == Pins[iCh])
        {
            channel = static_cast<channelType> (iCh);
            foundChannel = true;
        }
    }
    
    if (foundChannel)
    {
        minValue = minValues[channel];
        maxValue = maxValues[channel];
        value = minValue;
        
        pTable = new tableType;
        pTable->pTableTimes  = &signalTimes[channel][0];
        pTable->pTableValues = &signalValues[channel][0];
        pTable->tableLength  = lengthTable;
    }
    else
    {
        display("PwmIn::attach, channel not found.\n");
    }
}

int PwmIn::getPwm()
{
    if (pTable == NULL) return pwm;
    
    for (int i=0; i<pTable->tableLength; i++)
    {
        /*
         display("Time: ");
         display( getTime() );
         display(" Table Time: ");
         display( pTable->pTableTimes[i] );
         display(" Table Value: ");
         display( pTable->pTableValues[i] );
         display("\n");
         */
        if( getTime() >= pTable->pTableTimes[i] )
        {
            value = pTable->pTableValues[i];
        }
    }
    
    pwm = mapToPwm(value, minValue, maxValue, PWMMIN, PWMMAX);
    
    /*
    display("Channel: ");
    display((int) channel);
    display(", Value: ");
    display(value);
    display(", PWM: ");
    display(pwm);
    display("\n");
    */
    return pwm;
}

int PwmIn::mapToPwm(double value, double valueMin, double valueMax, int pwmMin, int pwmMax)
{
    // y - y1 = m * (x - x1)
    double slope = (pwmMax-pwmMin) / (valueMax-valueMin);
    return slope*(value-valueMin) + pwmMin;
}

#else
PwmIn::PwmIn()
{
    thisPin = -1;
    thisPinLoc = -1;
}

void PwmIn::attach(int pinIn)
{
    // Store Pin
    thisPin = pinIn;
    thisPinLoc = nPins;
    
    // Initialize Arrays
    pwm[thisPinLoc] = 1000;
    pinArray[thisPinLoc] = thisPin;
    
    // Setup Pin
    pinMode(thisPin, INPUT_PULLUP);
    
    // Initialize Variables
    if (nPins == 0)
    {
        iPin = 0;
        tRise = 0;
        enableInterrupt(pinArray[0], PwmIn::riseInterrupt, RISING);
    }
    
    // Increment Pins
    nPins++;
}
int PwmIn::getPwm()
{
    if (thisPinLoc != -1) { return pwm[thisPinLoc]; }
    
    // Calling getPwm before attached
    else { return 0; }
}

void PwmIn::riseInterrupt()
{
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        tRise = micros();
        enableInterrupt(pinArray[iPin], PwmIn::fallInterrupt, FALLING);
    }
}

void PwmIn::fallInterrupt()
{
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        int tempPWM = micros() - tRise;
        if (tempPWM >= minPWM && tempPWM <= maxPWM)
        {
            pwm[iPin] = tempPWM;
        }
        disableInterrupt(pinArray[iPin]);
        
        iPin++;
        if (iPin >= nPins) { iPin = 0; }
        enableInterrupt(pinArray[iPin], PwmIn::riseInterrupt, RISING);
    }
}
#endif

#ifndef SIMULATION
    int PwmIn::iPin  = 0;
    int PwmIn::nPins = 0;
    int PwmIn::pinArray[PwmIn::maxPins] = {0};
    volatile int PwmIn::tRise = 0;
    volatile int PwmIn::pwm[PwmIn::maxPins] = {0};
#endif
