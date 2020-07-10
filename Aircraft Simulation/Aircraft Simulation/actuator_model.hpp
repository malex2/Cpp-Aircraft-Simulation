//
//  actuator_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/5/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef actuator_model_hpp
#define actuator_model_hpp

#include "initial_conditions.hpp"

enum inputDirType {off, increasing, decreasing};

class ActuatorModel : public GenericModel
{
public:
    // Constructor
    ActuatorModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    ~ActuatorModel(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    float* getActuators(void) { return actuators; }
    
private:
    class DynamicsModel   *pDyn;
    class RotateFrame     *pRotate;
    class Time            *pTime;
    class PropulsionModel *pProp;
    
    class MonitorMacInput *pTerminal;
    class Delay           *pDebounce[nActuators];
    
    typedef GenericModel Base;
    
    // Functions
    void updateKeyboard(void);
    
    void updateSerial(void);
    
    void actuatorDynamics(void);
    
    void printKeyPress(int input);
    
    // Variables
    float actuators[nActuators];     // de, da, dr, dT
    
    // keyboard values
    float actuatorRatesRaw[nActuators]; // de, da, dr, dT
    float actuatorRates[nActuators]; // de, da, dr, dT
    keyType actuatorUpKey[nActuators];
    keyType actuatorDownKey[nActuators];
    float baseRate[nActuators];
    float maxRate[nActuators];
    float onDelay[nActuators];
    float offDelay[nActuators];
    
    inputDirType prevActuatorState[nActuators]; // off, increasing, decreasing
    inputDirType actuatorState[nActuators]; // off, increasing, decreasing
    inputDirType tempState[nActuators];
    
    // print variables
    float dActuators[nActuators];
    
    inputModeType inputMode;
};

#endif /* actuator_model_hpp */
