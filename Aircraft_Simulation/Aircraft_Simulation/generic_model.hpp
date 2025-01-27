//
//  general_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/12/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef general_model_hpp
#define general_model_hpp

#include <stdio.h>
#include <random>

#include "utilities.hpp"
#include "model_mapping.hpp"

class Time;

class GenericModel {
protected:
    // Classes
    Utilities util;
    ModelMap  *pMap;
    
    bool debugFlag;
    
    double dt;
    double prevTime;
    double time;
    int counter;
    
    virtual void updateDt(Time* pTime);
public:
    // Constructor
    GenericModel();
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(Time* pTime);
    
    // Update forces and moments
    virtual bool update(void) { return true; };
    
    virtual void setDebugFlag(bool flag);
};

class GenericForceModel : public GenericModel {
protected:
    // Forces and moments
    double LLForce[3];
    double bodyForce[3];
    double bodyMoment[3];
    
public:
    // Constructor
    GenericForceModel();
    
    // Return forces and moments
    double* getForce(void)   { return bodyForce; }
    double* getLLForce(void) { return LLForce; }
    double* getMoment(void)  { return bodyMoment; }
};

class GenericSensorModel : public GenericModel {
protected:
    class SimulationSerial* pSerialIO;
    
    bool perfectSensor;
    
    // Random Noise
    double randomNoise[3];
    double* randomNoiseModel(double* maxNoise);
    double randomNoiseModel(double maxNoise);
public:
    // Constructor
    GenericSensorModel();
    
    // Setters
    virtual void setPerfectSensor(bool input) { perfectSensor = input; }
    
    void setSerialIO(SimulationSerial*  pIO);
};

// Example class
class ExampleModel : public GenericForceModel
{
public:
    // Constructor
    ExampleModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    class Time          *pTime;
};

#endif /* general_model_hpp */
