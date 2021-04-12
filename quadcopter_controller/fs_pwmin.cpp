//
//  fs_pwmin.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_pwmin.hpp"

PwmIn throttleChannel;
PwmIn rollChannel;
PwmIn yawChannel;
PwmIn pitchChannel;

unsigned long pwmIn[nChannels];
double valuesIn[nChannels];

void FsPwmIn_setup()
{
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        pwmIn[iCh]    = PWMMIN;
        valuesIn[iCh] = 0.0;
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
    pwmIn[YAWRATE]  = yawChannel.getPwm();
    
    valuesIn[THROTTLE] = mapToValue(pwmIn[THROTTLE], PWMMIN, PWMMAX, 0     , MAXTHROTTLE); // Throttle (0 - 100)
    valuesIn[ROLL]     = mapToValue(pwmIn[ROLL]    , PWMMIN, PWMMAX, -MAXROLL   , MAXROLL);     // Roll (rad)
    valuesIn[PITCH]    = mapToValue(pwmIn[PITCH]   , PWMMIN, PWMMAX, -MAXPITCH  , MAXPITCH);    // Pitch (rad)
    valuesIn[YAWRATE]  = mapToValue(pwmIn[YAWRATE] , PWMMIN, PWMMAX, -MAXYAWRATE, MAXYAWRATE);  // Yaw Rate (rad/s)
    /*
    for (int iCh = THROTTLE; iCh != nChannels; iCh++)
    {
        display("Channel: ");
        display((int) iCh);
        display(", PWM: ");
        display(pwmIn[iCh]);
        display(", Value: ");
        display(valuesIn[iCh]);
        display("\n");
    }
     */
}

unsigned long* FsPwmIn_getPWM()    { return pwmIn; }
double*        FsPwmIn_getValues() { return valuesIn; }

#ifdef SIMULATION
PwmIn::PwmIn()
{
    // Initialize Pins
    Pins[THROTTLE] = THROTTLEPIN; // CH3
    Pins[ROLL]     = ROLLPIN;     // CH4
    Pins[PITCH]    = PITCHPIN;    // CH5
    Pins[YAWRATE]  = YAWPIN;      // CH6
    
    // Initialize Ranges
    minValues[THROTTLE] = 0.0;
    minValues[ROLL]     = -MAXROLL;
    minValues[PITCH]    = -MAXPITCH;
    minValues[YAWRATE]  = -MAXYAWRATE;
    
    maxValues[THROTTLE] = 100.0;
    maxValues[ROLL]     = MAXROLL;
    maxValues[PITCH]    = MAXPITCH;
    maxValues[YAWRATE]  = MAXYAWRATE;
    
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
            signalValues[iCh][col] *= deg2rad;
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

unsigned long PwmIn::getPwm()
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
unsigned long PwmIn::getPwm()
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
