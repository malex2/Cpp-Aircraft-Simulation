//
//  fs_barometer.cpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 8/15/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#include "fs_barometer.hpp"

// Barometer Data
BarometerType baroData;

// Bools
bool barometerSetup = false;
bool pressureInit   = false;

// State machine 
double stateTime;
double tempDelay;
double pressDelay;
bool   stateSetup;

// Calibration
double c5, c6, mc, md, x0, x1, x2, py0, py1, py2, p0, p1, p2;
byte pressureResolution;
double refPressure;
double altitudeRMS;

void FsBarometer_setupBarometer()
{
#ifdef BAROMETER
    short AC1;
    short AC2;
    short AC3;
    short VB1;
    short VB2;
    short MB;
    short MC;
    short MD;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    double c3;
    double c4;
    double b1;
    
    stateTime = 0.0;
    stateSetup = false;
    pressureResolution = BMP180_COMMAND_PRESSURE0;
    pressDelay = 5.0 / 1000.0;
    tempDelay = 5.0 / 1000.0;
    altitudeRMS = 0.5;
    
    // Calibrate
    AC1 = readCalVal(AC1_ADDR);
    AC2 = readCalVal(AC2_ADDR);
    AC3 = readCalVal(AC3_ADDR);
    AC4 = (unsigned short) readCalVal(AC4_ADDR);
    AC5 = (unsigned short) readCalVal(AC5_ADDR);
    AC6 = (unsigned short) readCalVal(AC6_ADDR);
    VB1 = readCalVal(VB1_ADDR);
    VB2 = readCalVal(VB2_ADDR);
    MB = readCalVal(MB_ADDR);
    MC = readCalVal(MC_ADDR);
    MD = readCalVal(MD_ADDR);
    
    // Compute floating-point polynominals:
    c3 = 160.0 * pow(2, -15) * AC3;
    c4 = pow(10, -3) * pow(2, -15) * AC4;
    b1 = pow(160, 2) * pow(2, -30) * VB1;
    c5 = (pow(2, -15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2, 11) / pow(160, 2)) * MC;
    md = MD / 160.0;
    x0 = AC1;
    x1 = 160.0 * pow(2, -13) * AC2;
    x2 = pow(160, 2) * pow(2, -25) * VB2;
    py0 = c4 * pow(2, 15);
    py1 = c4 * c3;
    py2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2, -20);
    p2 = 3038.0 * 100.0 * pow(2, -36);
    
    barometerSetup = true;
    
    display("Barometer setup\n");
#endif
}

short readCalVal(byte address)
{
    short value = 0;
    
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(address);
    baroData.errorCodeBaro = (I2C_Error_Code) Wire.endTransmission(false);
    
    Wire.requestFrom(BMP180_ADDR, 2, true);
    value = Wire.read() << 8 | Wire.read();
    
    return value;
}

void FsBarometer_setPressureResolution(byte pressureResolutionIn)
{
    pressureResolution = pressureResolutionIn;
    
    switch (pressureResolution)
    {
        case BMP180_COMMAND_PRESSURE0:
            pressDelay  = 5.0 / 1000.0;
            altitudeRMS = 0.5;
            break;
            
        case BMP180_COMMAND_PRESSURE1:
            pressDelay  = 8.0 / 1000.0;
            altitudeRMS = 0.4;
            break;
            
        case BMP180_COMMAND_PRESSURE2:
            pressDelay  = 14.0 / 1000.0;
            altitudeRMS = 0.3;
            break;
            
        case BMP180_COMMAND_PRESSURE3:
            pressDelay  = 26.0 / 1000.0;
            altitudeRMS = 0.25;
            break;
            
        default:
            display(getTime());
            display(" Invalid pressure resolution: ");
            display(pressureResolution);
            display("\n");
            
            pressureResolution = BMP180_COMMAND_PRESSURE0;
            pressDelay = 5.0 / 1000.0;
            break;
    }
    display("Barometer resolution set: ");
    display(+pressureResolution, HEX);
    display("\n");
}

void FsBarometer_performBarometer()
{
#ifdef BAROMETER
    if (!barometerSetup) { return; }
    
    if (baroData.state == startSequence)
    {
        baroData.state = requestTemperature;
        stateSetup = false;
    }
    
    if (baroData.state == requestTemperature)
    {
        if (!stateSetup)
        {
            stateTime = getTime();
            requestTemp();
            stateSetup = true;
        }
        
        if (getTime() - stateTime >= tempDelay)
        {
            stateSetup = false;
            baroData.state = readTemperature;
        }
    }
    
    if (baroData.state == readTemperature)
    {
        readTemp();
        stateSetup = false;
        baroData.state = requestPressure;
    }
    
    if (baroData.state == requestPressure)
    {
        if (!stateSetup)
        {
            stateTime = getTime();
            requestPres();
            stateSetup = true;
        }
        
        if (getTime() - stateTime >= pressDelay)
        {
            stateSetup = false;
            baroData.state = readPressure;
        }
    }
    
    if (baroData.state == readPressure)
    {
        readPres();
        
        if (!pressureInit)
        {
            refPressure = baroData.pressure;
            pressureInit = true;
        }
        
        computeAltitude();
        
        baroData.timestamp = getTime();
        
        stateSetup = false;
        baroData.state = baroReady;
    }
#endif
}

void requestTemp()
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_CONTROL);
    Wire.write(BMP180_COMMAND_TEMPERATURE);
    baroData.errorCodeBaro = (I2C_Error_Code) Wire.endTransmission(true);
}

void readTemp()
{
    double tu;
    double a;
    
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_RESULT);
    baroData.errorCodeBaro = (I2C_Error_Code) Wire.endTransmission(false);
    Wire.requestFrom(BMP180_ADDR, 2, true);
    
    tu = (Wire.read() * 256.0) + Wire.read();
    a = c5 * (tu - c6);
    
    baroData.tu = tu;
    baroData.temperature = a + (mc / (a + md));
}

void requestPres()
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_CONTROL);
    Wire.write(pressureResolution);
    baroData.errorCodeBaro = (I2C_Error_Code) Wire.endTransmission(true) ;
}

void readPres()
{
    double pu;
    double s;
    double x;
    double y;
    double z;
    
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_RESULT);
    baroData.errorCodeBaro = (I2C_Error_Code) Wire.endTransmission(false);
    Wire.requestFrom(BMP180_ADDR, 3, true);
    
    pu = (Wire.read() * 256.0) + Wire.read() + (Wire.read() / 256.0);
    
    s = baroData.temperature - 25.0;
    x = (x2 * s * s) + (x1 * s) + x0;
    y = (py2 * s * s) + (py1 * s) + py0;
    z = (pu - x) / y;
    
    baroData.pu = pu;
    baroData.pressure = (p2 * z * z) + (p1 * z) + p0;
}

void computeAltitude()
{
    baroData.altitude = 44330.0 * ( 1.0 - pow(baroData.pressure / refPressure, 1.0 / 5.255) );
}

bool FsBarometer_startBarometerMeasurement()
{
    if (baroData.state == baroStandby)
    {
        baroData.state = startSequence;
        stateTime = getTime();
        stateSetup = false;
        return true;
    }
    else
    {
#ifdef BAROMETER
        display(getTime());
        display(" Barometer not ready\n");
#endif
        return false;
    }
}

void FsBarometer_setStandby()
{
    baroData.state = baroStandby;
}

BarometerType* FsBarometer_getBaroData() { return &baroData; }

baroStateType FsBarometer_getBaroState() { return baroData.state; }

double FsBarometer_getAltitudeVariance() { return altitudeRMS*altitudeRMS; }
