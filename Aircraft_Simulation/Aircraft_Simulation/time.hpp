//
//  Time.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef Time_hpp
#define Time_hpp

#include <stdio.h>
#include "initial_conditions.hpp"
#include "generic_model.hpp"

class Time : public GenericModel {

public:
    // Constructor
    Time(ModelMap *pMapInit, bool debugFlagIn = false );
    ~Time();
    
    // Getters
    systemTime getSystemTime()      { return systemtime; }
    double     getSimTime()         { return curTime; }
    bool       performPrint()       { return print; }
    bool       performSave()        { return save; }
    bool       performDynamics()    { return dynamics; }
    int        getCount()           { return counter; }      // get counter
    int        getClockCount()      { return clockCounter; } // get counter at clock_dt rate
    double     getDeltaTime()       { return dt; }           // get delta time of simulation
    
    // GPS Getters
    int getGPSTime()       { return pGPSTime->getGPSTime(); }
    int getGPSTimeOfWeek() { return pGPSTime->getGPSTimeOfWeek(); }
    int getUTCMonth()      { return pGPSTime->getUTCMonth(); }
    int getUTCDay()        { return pGPSTime->getUTCDay(); }
    int getUTCYear()       { return pGPSTime->getUTCYear(); }
    int getUTCTimeOfDay()  { return pGPSTime->getUTCTimeOfDay(); }
    int getUTCHour()       { return pGPSTime->getUTCHour(); }
    int getUTCMinute()     { return pGPSTime->getUTCMinute(); }
    int getUTCSeconds()    { return pGPSTime->getUTCSeconds(); }
    
    // Functions
    virtual bool update(void);
    
private:
    typedef GenericModel Base;
    
    systemTime systemtime;
    systemTime startTime;
    systemTime lastDynamicTime;
    systemTime lastPrintTime;
    systemTime lastSaveTime;
    systemTime lastClockDt;
    
    double timeSinceLastDynamics;
    double timeSinceLastPrint;
    double timeSinceLastSave;
    double timeSinceLastClockDt;
    
    double curTime;
    double runTime;
    double dynamicsInterval;
    double printInterval;
    double saveInterval;
    const double dtPad = 1e-10;
    GPSTimeType* pGPSTime;
    
    // Getters
    bool print;
    bool save;
    bool dynamics;
    int  counter;
    int  clockCounter;
};

#endif /* Time_hpp */
