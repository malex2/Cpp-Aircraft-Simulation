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
    float getSimTime(void)     { return curTime; }
    bool performPrint(void)    { return print;    };
    bool performSave(void)     { return save;     };
    bool performDynamics(void) { return dynamics; };
    
    // Functions
    virtual bool update(void);
    
private:
    systemTime systemtime;
    systemTime startTime;
    systemTime lastDynamicTime;
    systemTime lastPrintTime;
    systemTime lastSaveTime;
    
    double timeSinceLastDynamics;
    double timeSinceLastPrint;
    double timeSinceLastSave;
    
    float curTime;
    float runTime;
    float dynamicsInterval;
    float printInterval;
    float saveInterval;
    
    // Getters
    bool print;
    bool save;
    bool dynamics;
};

#endif /* Time_hpp */
