//
//  Time.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include <iostream>

#include "model_mapping.hpp"
#include "time.hpp"

Time::Time(ModelMap *pMapInit, bool debugFlagIn)
{
    pMap = pMapInit;
    
    pMap->addLogVar("Time ", &curTime, printSavePlot, 3);
    
    // Start clock
#ifdef RealTime
    startTime = std::chrono::system_clock::now();
    curTime   = 0.0;
#else
    startTime = 0.0;
    curTime   = 0.0;
#endif
    systemtime       = startTime;
    lastPrintTime    = startTime;
    lastSaveTime     = startTime;
    lastDynamicTime  = startTime;
    lastClockDt      = startTime;
    
    timeSinceLastDynamics = 0.0;
    timeSinceLastPrint    = 0.0;
    timeSinceLastSave     = 0.0;
    timeSinceLastClockDt  = 0.0;
    
    // Update time constants
    runTime          = runTime_init;
    printInterval    = printInterval_init;
    saveInterval     = saveInterval_init;
    dynamicsInterval = dynamicsInterval_init;
    
    pGPSTime = new GPSTimeType(month_init, day_init, year_init, hour_init, minute_init, second_init);
    
    print    = true;
    save     = true;
    dynamics = true;
    
    counter = 0;
    clockCounter = 0;
    
    debugFlag = debugFlagIn;
}

Time::~Time()
{
    delete pGPSTime;
}

bool Time::update(void)
{
    Base::updateDt(this);
    
#ifdef RealTime
    // Update time
    systemtime = std::chrono::system_clock::now();

    // Update elapsed times
    timeDuration durationTemp;
    
    durationTemp = systemtime - lastDynamicTime;
    timeSinceLastDynamics = durationTemp.count();
    
    durationTemp = systemtime - lastPrintTime;
    timeSinceLastPrint = durationTemp.count();
    
    durationTemp = systemtime - lastSaveTime;
    timeSinceLastSave = durationTemp.count();
    
    durationTemp = systemtime - startTime;
    curTime = durationTemp.count();
    
    durationTemp = systemtime - lastClockDt;
    timeSinceLastClockDt = durationTemp.count();
#else
    // Update time
    systemtime += clock_dt;
    curTime    += clock_dt;
    
    // Update elapsed times
    timeSinceLastDynamics = systemtime - lastDynamicTime;
    timeSinceLastPrint    = systemtime - lastPrintTime;
    timeSinceLastSave     = systemtime - lastSaveTime;
    timeSinceLastClockDt  = systemtime - lastClockDt;
#endif
    
    timeSinceLastDynamics += dtPad;
    timeSinceLastPrint    += dtPad;
    timeSinceLastSave     += dtPad;
    timeSinceLastClockDt  += dtPad;
    
    if (timeSinceLastDynamics >= dynamicsInterval)
    {
        lastDynamicTime = systemtime;
        dynamics = true;
    }
    else { dynamics = false; }
    
    if (timeSinceLastPrint >= printInterval)
    {
        lastPrintTime = systemtime;
        print = true;
    }
    else { print = false; }
    
    if (timeSinceLastSave >= saveInterval)
    {
        lastSaveTime = systemtime;
        save = true;
    }
    else { save = false; }
    
    if (timeSinceLastClockDt >= clock_dt)
    {
        pGPSTime->update(timeSinceLastClockDt);
        lastClockDt = systemtime;
        clockCounter++;
    }
    
    counter++;
    
    // Udpate stop criteria
    if ( curTime > runTime )
    {
        printf("Run time reached.\n");
        return false;
    }
    else { return true; }
}

