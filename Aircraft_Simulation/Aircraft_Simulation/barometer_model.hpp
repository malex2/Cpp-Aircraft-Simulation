//
//  barometer_model.hpp
//  Aircraft_Simulation
//
//  Created by Alexander McLean on 8/27/21.
//  Copyright © 2021 Alexander McLean. All rights reserved.
//

#ifndef barometer_model_hpp
#define barometer_model_hpp

#include "generic_model.hpp"

class BarometerModelBase : public GenericSensorModel
{
public:
    // Constructor
    BarometerModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    virtual double getPressure()    { return pressureSensor; }
    virtual double getTemperature() { return temperatureSensor; }
    
    virtual void setPerfectSensor(bool input) { perfectSensor = input; }
protected:
    virtual void updatePressure();
    virtual void updateTemperature();
    
    class DynamicsModel   *pDyn;
    class AtmosphereModel *pAtmo;
    class Time            *pTime;
    
    // Enviromental Variables
    double pressure;
    double temperature;
    double pressureCheck;
    
    // Sensor Variables
    bool perfectSensor;
    
    double pressureSensor;
    double temperatureSensor;
    
    // Noise
    double pressureNoiseMax;
    double pressureError;
    uniformNoiseM pressureNoise;
    
    double temperatureNoiseMax;
    double temperatureError;
    uniformNoiseM temperatureNoise;
};

class bmp180 : public BarometerModelBase
{
public:
    enum calibrationType {AC1, AC2, AC3, AC4, AC5, AC6, VB1, VB2, MB, MC, MD, nCalibrationData};
    enum requestStateType {PRESSURE, TEMPERATURE, NOREQUEST};
    
    bmp180(ModelMap *pMapInit, bool debugFlagIn);
    
    virtual bool update(void);
    
    virtual double getTemperature() { tempReading.endReading(); lastRequest = NOREQUEST; return temperatureSensor; }
    virtual double getPressure()    { presReading.endReading(); lastRequest = NOREQUEST; return pressureSensor; }
    
    requestStateType getLastRequest() { return lastRequest; }
    
    // Called by master device
    void startTemperatureReading() { tempReading.startReading(); lastRequest = TEMPERATURE; }
    void startPressureReading()    { presReading.startReading(); lastRequest = PRESSURE; }
    void setPressureNoise(int);
    
    int* getCalibrationParameter() { return calData; };
private:
    typedef BarometerModelBase Base;
    
    // Functions
    virtual void updateTemperature();
    virtual void updatePressure();
    
    // Data Types
    class sensorReadingType {
    public:
        sensorReadingType()
        {
            readingSum      = 0.0;
            readingCount    = 0;
            readDelay       = 0.0;
            readCommandTime = 0.0;
            sensorReading   = 0.0;
            readState = NOCOMMAND;
        }
        
        void setDelay(double delayIn) { readDelay = delayIn; }
        
        double update(double time, double data)
        {
            if (readState == STARTREADING)
            {
                readCommandTime = time;
                readState = READING;
            }
            
            if (readState == READING)
            {
                //readingSum += data;
                //readingCount++;
                if ((time - readCommandTime) > readDelay)
                {
                    //sensorReading = (double) readingSum / readingCount;
                    sensorReading = data;
                    readState = DONEREADING;
                    
                    readingSum = 0.0;
                    readingCount = 0;
                }
            }
            
            return sensorReading;
        }

        void startReading()
        {
            if (readState != NOCOMMAND) { std::cout << "Warning - not done reading sensor" << std::endl; }
            readState = STARTREADING;
        }
        
        void endReading()
        {
            if (readState != DONEREADING) { std::cout << "Warning - sensor data not ready! Currently in state " << readState << std::endl; }
            readState = NOCOMMAND;
        }

    private:
        enum readingStateType {STARTREADING, READING, DONEREADING, NOCOMMAND};
        
        double readingSum;
        int readingCount;
        double readDelay;
        double readCommandTime;
        double sensorReading;
        readingStateType readState;
    };
    
    // Sensor Readings
    sensorReadingType tempReading;
    sensorReadingType presReading;
    requestStateType lastRequest;
    
    // pressure and temperature
    double pu;
    double tu;

    // Calibration constants
    int calData[nCalibrationData];
    double c5, c6, mc, md, x0, x1, x2, y0, y1, y2, p0, p1, p2;
    
    // Noise
    double pressureNoise0;
    double pressureNoise1;
    double pressureNoise2;
    double pressureNoise3;
    
    double temperatureDelay;
    double pressureDelay0;
    double pressureDelay1;
    double pressureDelay2;
    double pressureDelay3;
    double pressureDelay;
};

#endif /* barometer_model_hpp */
