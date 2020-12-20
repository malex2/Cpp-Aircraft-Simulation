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

struct tableType
{
    double *pTableValues;
    double *pTableTimes;
    int    tableLength;
};

// **********************************************************************
// Actuator Model Base
// Define number and type of actuators, call their update functions
// **********************************************************************
class ActuatorModelBase : public GenericModel
{
public:
    // Constructor
    ActuatorModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Deconstructor
    ~ActuatorModelBase(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);

    // Update obejct states and forces
    virtual bool update(void);

    const static int maxActuators   = 6;
    
    // Getters
    virtual double* getActuators(void) { return positions; }
    
    // Setters
    virtual void setCommands(double* commands_in) { util.setArray(commands, commands_in, maxActuators); }
    
    bool continueArray[maxActuators];
    
protected:
    class DynamicsModel       *pDyn;
    class RotateFrame         *pRotate;
    class Time                *pTime;
    class PropulsionModelBase *pProp;
    class AeroModelBase       *pAero;
    class AtmosphereModel     *pAtmo;
    
    class MonitorMacInput  *pTerminal;
    class ActuatorTypeBase *actuators[maxActuators];
    
    // Variables
    double positions[maxActuators];
    double commands[maxActuators];
    tableType *pTable[maxActuators];
    
};

// **********************************************************************
// RC Plane Actuator Model
// **********************************************************************
class RCActuatorModel : public ActuatorModelBase
{
public:
    RCActuatorModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    enum actuatorType {de, da, dr, dT, nActuators};
    
private:
    // Tables
    static const int lengthTable = 3;
    
    double actuatorTimes[nActuators][lengthTable] = {
        {0, 5, 10},
        {0, 5, 10},
        {0, 5, 10},
        {0, 5, 10}
    };
    
    double actuatorValues[nActuators][lengthTable] = {
        {0  ,0, 0},
        {0  , 0  , 0},
        {0  , 0  , 0},
        {0.3, 0.5, 0.2}
    };
    
    // Servo dynamics
    double servoTau[3]     = {0.01, 0.01, 0.01};
    double servoMaxRate[3] = {45.0, 45.0, 45.0};
};

// **********************************************************************
// Quadcopter Actuator Model
// **********************************************************************
class QuadcopterActuatorModel : public ActuatorModelBase
{
public:
    QuadcopterActuatorModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    enum actuatorType {T1, T2, T3, T4, nActuators};
    
private:
    // Tables
    /*
    static const int lengthTable = 4;
    
    double actuatorTimes[nActuators][lengthTable] = {
        {0, 5, 10, 12},
        {0, 5, 10, 12},
        {0, 5, 10, 12},
        {0, 5, 10, 12}
    };
    
    double actuatorValues[nActuators][lengthTable] = {
        {0, 0.8, 0.8, 1.0}, // front left
        {0, 0.8, 0.8, 1.0}, // front right
        {0, 0.8, 0.8, 1.0}, // rear right
        {0, 0.8, 0.8, 1.0}  // rear left
    };
    */
    
    static const int lengthTable = 8;
     
    double actuatorTimes[nActuators][lengthTable] = {
        {0, 5, 10, 10.15, 10.3, 29, 29.15, 29.3},
        {0, 5, 10, 10.15, 10.3, 29, 29.15, 29.3},
        {0, 5, 10, 10.15, 10.3, 29, 29.15, 29.3},
        {0, 5, 10, 10.15, 10.3, 29, 29.15, 29.3}
    };
    
    double actuatorValues[nActuators][lengthTable] = {
        {0, 0.8, 0.805 , 0.795, 0.8, 0.795, 0.805, 0.8}, // front left
        {0, 0.8, 0.805 , 0.795, 0.8, 0.795, 0.805, 0.8}, // front right
        {0, 0.8, 0.795 , 0.805, 0.8, 0.805, 0.795, 0.8}, // rear right
        {0, 0.8, 0.795 , 0.805, 0.8, 0.805, 0.795, 0.8}  // rear left
    };
    
    // Servo dynamics
    double escDelays[4] = {0.01, 0.01, 0.01, 0.01};
};

// **********************************************************************
// Actuator Type Base
// **********************************************************************
class ActuatorTypeBase : public GenericModel
{
public:
    // Input Modes
    enum inputModeType {external, keyboard, table};
    inputModeType inputMode;
    
    // Constructor
    ActuatorTypeBase(ModelMap *pMapInit, bool debugFlagIn = false, double init_position = 0.0);
 
    // Deconstructor
    ~ActuatorTypeBase(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Table controls
    tableType *pTable;
    
    // Functions
    void updateKeyboard(void);
    void updateTable(void);
    virtual void updateDynamics(void);
    virtual void applyLimits(void);
    
    // Setters
    virtual void setCommand(double cmd_in)                    { command = cmd_in; }
    virtual double getCommand(void)                           { return command; }
    virtual void setLimits(double lower, double upper)         { lowerLimit = lower; upperLimit = upper; limitsSet = true;}
    virtual void setTerminal(MonitorMacInput *pTerminalIn)   { terminal->pTerminal = pTerminalIn; inputMode = keyboard; }
    virtual void setKeys(keyType upKeyIn, keyType downKeyIn) { terminal->upKey = upKeyIn; terminal->downKey = downKeyIn; }
    virtual void setDebounce(Delay *delay)                   { terminal->pDebounce = delay; }
    virtual void setDelay(double onDelay, double offDelay)     { terminal->onDelay = onDelay; terminal->offDelay = offDelay; }
    virtual void setRate(double baseRate, double maxRate)      { terminal->baseRate = baseRate; terminal->maxRate = maxRate; }
    virtual void setTable(tableType* tableIn)                { pTable = tableIn;  inputMode = table; }
    
    // Getters
    virtual double getPosition(void) { return position; }
    
    // Terminal controls
    class terminalType
    {
    public:
        terminalType(bool debugFlag = false, MonitorMacInput *pTerminal = NULL, Delay *pDebounce = NULL, keyType upKey = upArrow, keyType downKey = downArrow, double baseRate = 1.0, double maxRate = 2.0, double onDelay = 0.0, double offDelay = 0.2)
        {
            prevActuatorState = off;
            actuatorState     = off;
            tempState         = off;
            actuatorRateRaw   = 0.0;
            actuatorRate      = 0.0;
        }
        
        ~terminalType(void)
        {
            if (pDebounce != NULL) { delete pDebounce; }
        }
        
        enum inputDirType {off, increasing, decreasing}; // direction from terminal
        
        class MonitorMacInput  *pTerminal;
        class Delay *pDebounce;
        
        keyType upKey;
        keyType downKey;
        
        double baseRate;
        double maxRate;
        double onDelay;
        double offDelay;
        
        // Functions
        void  update(void);
        void  printKeyPress(int);
        double getRate(void) { return actuatorRate; }
        
    private:
        inputDirType prevActuatorState;
        inputDirType actuatorState;
        inputDirType tempState;
        
        double actuatorRateRaw;
        double actuatorRate;
        
        bool debugFlag;
    };
    
    terminalType *terminal;
    
protected:
    class Time *pTime;
    typedef GenericModel Base;
    
    // Variables
    double command;
    double position;
    double lowerLimit; // lower position limit
    double upperLimit; // upper position limit
    bool  limitsSet;
};

// **********************************************************************
// Servo
// **********************************************************************
class Servo : public ActuatorTypeBase
{
public:
    Servo(ModelMap *pMapInit, bool debugFlagIn = false, double init_position = 0.0);
    
    void setTimeConstant(double tau_in) { positionDynamics.setTimeConstant(tau_in); }
    void setDt(double dt_in)            { positionDynamics.setDt(dt_in); }
    void setMaxRate(double maxrate_in)  { positionDynamics.setMaxRate(maxrate_in); }
    
private:
    FirstOrderFilter positionDynamics;

    typedef ActuatorTypeBase Base;
    
    virtual void updateDynamics(void);
    // virtual void applyLimits(void);
};

// **********************************************************************
// Brushless DC Motor
// **********************************************************************
class BrushlessDCMotor : public ActuatorTypeBase
{
public:
    BrushlessDCMotor(ModelMap *pMapInit, bool debugFlagIn = false, double init_position = 0.0);
    ~BrushlessDCMotor(void);
    
    void setESCDelay(double delay_in) { pESC->setDelay(delay_in); }
    
private:
    typedef ActuatorTypeBase Base;
    
    virtual void updateDynamics(void);
    // virtual void applyLimits(void);
    
    Delay *pESC;
    
};

#endif /* actuator_model_hpp */
