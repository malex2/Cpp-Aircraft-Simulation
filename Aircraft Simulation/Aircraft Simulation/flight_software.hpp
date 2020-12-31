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
#include <iostream>

class ModelMap;

// Main Functions
bool mainFlightSoftware(void);
void getImuData(double* acc, double* gyro, double* temperature);
bool groundCalibration(double* accBias, double* gyroBias, double* acc, double* gyro);

 // Compute  roll, pitch, yaw
void atittudeFilter(double* attitude, double* acc, double* gyro);

// Determine desired roll, pitch, and yaw
// Compute de, da, dr through PID
// Set each throttle value
void attitudeControl(double* attitude);

// Initialization Functions
void initialize(void);
void initializeVariables(void);
void getSimulationModels(void);
void setupIMU(void);

// Setters
void flightSoftware_setMapPointer(ModelMap* pMapInit);

// Printing
void printStatement(std::string val);
template<typename TempType>
void printStatement(TempType val);

#endif /* flight_software_hpp */
