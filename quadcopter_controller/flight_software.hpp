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

// Main Functions
bool mainFlightSoftware(void);

// IMU
void IMUInterrupt(void);
void getImuData(double* acc, double* gyro, double* temperature);
bool groundCalibration(double* accBias, double* gyroBias, double* acc, double* gyro);

// GPS
void GPSInterrupt(void);

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
void setupArduino(void);

// Setters
void flightSoftware_setMapPointer(ModelMap* pMapInit);

// Errors
void computeErrors(void);

// Time
double getTime(void);

// Printing
//void display(String val);
template<typename TempType>
void display(TempType val);

// Interrupts
class PwmIn {
#ifdef SIMULATION
public:
    // Constructor
    PwmIn();
    
    void attach(int pinIn);
    
    unsigned long getPwm();
private:
    // Types
    enum channelType {THROTTLE, ROLL, PITCH, YAWRATE, nChannels};
    
    struct tableType
    {
        double *pTableValues;
        double *pTableTimes;
        int    tableLength;
    };
    tableType *pTable;
    
    // Variables
    unsigned int Pins[nChannels];
    double minValues[nChannels];
    double maxValues[nChannels];
    
    channelType channel;
    unsigned long pwm;
    unsigned long value;
    double minValue;
    double maxValue;
    bool foundChannel;
    
    // Tables
    static const int lengthTable = 4;
    
    double signalTimes[nChannels][lengthTable] = {
        {0, 5, 7, 10},
        {0, 5, 7, 10},
        {0, 5, 7, 10},
        {0, 5, 7, 10}
    };
    
    double signalValues[nChannels][lengthTable] = {
        {0, 20, 20, 20}, // Throttle (%)
        {0, 0 , 5 , -5}, // Aileron  (deg)
        {0, 0 , 0 , 0 }, // Pitch    (deg)
        {0, 0 , 0 , 0 }  // Yaw      (deg)
    };
#else
public:
    // Constructor
    PwmIn();
    
    // Functions
    void attach(int pinIn);
    
    // Instance Getters
    unsigned long getPwm();
    int getPin()    { return thisPin; }
    int getPinLoc() { return thisPinLoc; }
    static int getCurPin() { return pinArray[iPin]; }

    // Interrupts
    static void riseInterrupt();
    static void fallInterrupt();

    // Constants
    static const int maxPins = 6;
    static const int minPWM = 995;
    static const int maxPWM = 2005;
private:
    // Instance Info
    int thisPin;
    int thisPinLoc;
    
    // Pin Info
    static int iPin;  // Position of pin in pinArray
    static int nPins; // Number of interrupt pins
    static int pinArray[maxPins];     // Array of interrupt pins
    static volatile int tRise;        // Rise time for current interrupt
    static volatile int pwm[maxPins]; // Array of interrupt pwms
#endif
};

// Servo
#ifdef SIMULATION
class Servo {
public:
    Servo();
    
    void attach(int pinIn);
    
    void writeMicroseconds(unsigned long pwm);
private:
    // Types
    enum motorNumberType {T1, T2, T3, T4, nMotors};
    
    // Variables
    unsigned int motorPins[nMotors];
    motorNumberType motorNumber;
    bool foundMotor;
    double throttle;
};
#endif

// LED
void LEDon();
void LEDoff();

// Mapping
double mapToValue(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax, double valueMin, double valueMax);
unsigned long mapToPwm(double value, double valueMin, double valueMax, unsigned long pwmMin, unsigned long pwmMax);
unsigned long limit(unsigned long pwm, unsigned long pwmMin, unsigned long pwmMax);

#endif /* flight_software_hpp */
