//
//  fs_barometer.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 8/15/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef fs_barometer_hpp
#define fs_barometer_hpp

#include "fs_common.hpp"

// Data Types
enum baroStateType {startSequence, requestTemperature, readTemperature, requestPressure, readPressure, baroReady, baroStandby, nBaroStates};

struct BarometerType {
    double pressure;
    double temperature;
    double altitude;
    baroStateType state;
    double timestamp;
    I2C_Error_Code errorCodeBaro;
    
    // temp vars
    double pu, tu;
    BarometerType()
    {
        pressure    = 0.0;
        temperature = 0.0;
        altitude    = 0.0;
        state       = baroStandby;
        timestamp   = 0.0;
        errorCodeBaro = I2C_0_SUCCESS;
    }
};

// Barometer Setup
void FsBarometer_setupBarometer();

// Barometer Update
void FsBarometer_performBarometer();
void requestTemp();
void readTemp();
void requestPres();
void readPres();
void computeAltitude();

// Support
short readCalVal(byte address);

// Setters
bool FsBarometer_startBarometerMeasurement();
void FsBarometer_setStandby();
void FsBarometer_setPressureResolution(byte pressureResolutionIn);

// Getters
BarometerType* FsBarometer_getBaroData();
baroStateType FsBarometer_getBaroState();
double FsBarometer_getAltitudeVariance();
#endif /* fs_barometer_hpp */
