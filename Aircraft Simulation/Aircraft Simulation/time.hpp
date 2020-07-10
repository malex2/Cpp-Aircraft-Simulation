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
    
    // Getters
    systemTime getSystemTime(void) { return systemtime; }
    float getSimTime(void)         { return curTime; }
    bool  performPrint(void)       { return print; }
    bool  performSave(void)        { return save; }
    bool  performDynamics(void)    { return dynamics; }
    int   getCount(void)           { return counter; } // get counter at clock_dt rate
    float getDeltaTime(void)       { return dt; }      // get delta time of simulation
    
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
    
    float curTime;
    float runTime;
    float dynamicsInterval;
    float printInterval;
    float saveInterval;
    
    // Getters
    bool print;
    bool save;
    bool dynamics;
    int  counter;
};

#endif /* Time_hpp */
