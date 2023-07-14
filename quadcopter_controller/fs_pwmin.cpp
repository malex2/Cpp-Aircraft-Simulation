//
//  fs_pwmin.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 3/14/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_pwmin.hpp"

# ifndef SIMULATION
#ifdef PWM
    #define EI_ARDUINO_INTERRUPTED_PIN
    #include <EnableInterrupt.h>
#endif
#endif

PwmIn throttleChannel;
PwmIn rollChannel;
PwmIn yawChannel;
PwmIn pitchChannel;

int pwmIn[nChannels];
unsigned long validReads;
bool pwmValid;
bool pwmSetup = false;;

void FsPwmIn_setup()
{
#ifdef PWM
    pwmIn[THROTTLE_CHANNEL] = PWMMIN;
    pwmIn[ROLL_CHANNEL]     = (PWMMIN+PWMMAX)/2;
    pwmIn[PITCH_CHANNEL]    = (PWMMIN+PWMMAX)/2;
    pwmIn[YAW_CHANNEL]      = (PWMMIN+PWMMAX)/2;
    
    throttleChannel.attach(THROTTLEPIN);
    rollChannel.attach(ROLLPIN);
    pitchChannel.attach(PITCHPIN);
    yawChannel.attach(YAWPIN);
    
    validReads = 0;
    pwmValid   = false;
    pwmSetup   = true;
#endif
}

void FsPwmIn_performPwmIn()
{
#ifdef PWM
    pwmValid = false;
    if (!pwmSetup) { return; }
    
    // Get PWM readings
    pwmIn[THROTTLE_CHANNEL] = throttleChannel.getPwm();
    pwmIn[ROLL_CHANNEL]     = rollChannel.getPwm();
    pwmIn[PITCH_CHANNEL]    = pitchChannel.getPwm();
    pwmIn[YAW_CHANNEL]      = yawChannel.getPwm();
    
    // Determine if readings are valid
    if(PwmIn::getValidReadCount() > validReads)
    {
        validReads = PwmIn::getValidReadCount();
        pwmValid = true;
    }
#endif
}

const int* FsPwmIn_getPWM() { return pwmIn; }
bool FsPwmIn_valid()  { return pwmValid; }

#ifdef SIMULATION
PwmIn::PwmIn()
{
    // Initialize Pins
    Pins[THROTTLE_CHANNEL] = THROTTLEPIN; // CH3
    Pins[ROLL_CHANNEL]     = ROLLPIN;     // CH4
    Pins[PITCH_CHANNEL]    = PITCHPIN;    // CH5
    Pins[YAW_CHANNEL]      = YAWPIN;      // CH6
    
    // Initialize Ranges
    
    // Velocity Control
    minValues[THROTTLE_CHANNEL] = -MAXVELOCITY;
    minValues[ROLL_CHANNEL]     = -MAXVELOCITY;
    minValues[PITCH_CHANNEL]    = -MAXVELOCITY;
    minValues[YAW_CHANNEL]      = -MAXYAWRATE;
    
    maxValues[THROTTLE_CHANNEL] = MAXVELOCITY;
    maxValues[ROLL_CHANNEL]     = MAXVELOCITY;
    maxValues[PITCH_CHANNEL]    = MAXVELOCITY;
    maxValues[YAW_CHANNEL]      = MAXYAWRATE;
    
    // Initialize Variables
    channel  = THROTTLE_CHANNEL;
    pwm      = PWMMIN;
    minValue = minValues[channel];
    maxValue = maxValues[channel];
    value    = minValue;
    pTable   = NULL;
    foundChannel = false;
    
    // Table To Radians
    for (int iCh = YAW_CHANNEL; iCh != nChannels; iCh++)
    {
        for (int col = 0; col < lengthTable; col++)
        {
            signalValues[iCh][col] *= degree2radian;
        }
    }
}

PwmIn::~PwmIn()
{
    if (pTable) { delete pTable; pTable = NULL;}
}

void PwmIn::attach(int pinIn)
{
    // Find Channel
    for (int iCh = THROTTLE_CHANNEL; iCh != nChannels; iCh++)
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
        if( getTime() >= pTable->pTableTimes[i] )
        {
            value = pTable->pTableValues[i];
        }
    }
    
    pwm = mapToPwm(value, minValue, maxValue, PWMMIN, PWMMAX);
    
    if (channel == YAW_CHANNEL) { validReadCount++; }
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
#ifdef PWM
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
#endif
}
int PwmIn::getPwm()
{
    if (thisPinLoc != -1) { return pwm[thisPinLoc]; }
    
    // Calling getPwm before attached
    else { return 0; }
}

void PwmIn::riseInterrupt()
{
#ifdef PWM
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        tRise = micros();
        enableInterrupt(pinArray[iPin], PwmIn::fallInterrupt, FALLING);
    }
#endif
}

void PwmIn::fallInterrupt()
{
#ifdef PWM
    if (arduinoInterruptedPin == pinArray[iPin])
    {
        int tempPWM = micros() - tRise;
        if (tempPWM >= minPWM && tempPWM <= maxPWM)
        {
            pwm[iPin] = tempPWM;
        }
        disableInterrupt(pinArray[iPin]);
        
        iPin++;
        if (iPin >= nPins) { iPin = 0; validReadCount++; }
        enableInterrupt(pinArray[iPin], PwmIn::riseInterrupt, RISING);
    }
#endif
}
#endif

unsigned long PwmIn::validReadCount = 0;
#ifndef SIMULATION
    int PwmIn::iPin  = 0;
    int PwmIn::nPins = 0;
    int PwmIn::pinArray[PwmIn::maxPins] = {0};
    volatile int PwmIn::tRise = 0;
    volatile int PwmIn::pwm[PwmIn::maxPins] = {0};
#endif
