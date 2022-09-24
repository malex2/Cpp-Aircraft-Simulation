//
//  flight_software.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef flight_software_hpp
#define flight_software_hpp

#include "fs_common.hpp"

class ModelMap;

// Initialization Functions
void initialize();
void initializeRoutines();
void initializeTime();
void getModels();
#ifdef SIMULATION
    void setPrintVariables();
#endif
void setupIO();

// Main Function
bool mainFlightSoftware();

// Functions
void performSerialIO();

// Setters
#ifdef SIMULATION
    void flightSoftware_setMapPointer(ModelMap* pMapInit);
#endif

// Support
void printData();

#endif /* flight_software_hpp */
