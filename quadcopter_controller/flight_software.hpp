//
//  flight_software.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/30/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef flight_software_hpp
#define flight_software_hpp

#define SIMULATION

#include <stdio.h>
#ifdef SIMULATION
    #include <iostream>
    typedef std::string String;
#else
    #include "arduino.h"
#endif

class ModelMap;

// Initialization Functions
void initialize();
void initializeVariables();
void getSimulationModels();
void setPrintVariables();
void setup();

// Main Function
bool mainFlightSoftware();

// Setters
void flightSoftware_setMapPointer(ModelMap* pMapInit);

// Errors
void computeErrors();

// Interrupts


#endif /* flight_software_hpp */
