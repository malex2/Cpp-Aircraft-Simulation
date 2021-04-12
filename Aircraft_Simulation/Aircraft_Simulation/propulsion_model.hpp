//
//  propulsion_model.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 6/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef propulsion_model_hpp
#define propulsion_model_hpp

#include "generic_model.hpp"

// **********************************************************************
// Propulsion Model Base
// **********************************************************************
class PropulsionModelBase : public GenericForceModel
{
public:
    // Constructor
    PropulsionModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
 
    // Deconstructor
    ~PropulsionModelBase(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    virtual void updatePropulsorStates(void);
    
    const static int maxPropulsors = 10;
    
protected:
    class DynamicsModel     *pDyn;
    class RotateFrame       *pRotate;
    class ActuatorModelBase *pAct;
    class Time              *pTime;
    class PropulsionTypeBase *pPropulsors[maxPropulsors];
};

// **********************************************************************
// RC Plane Propulsion Model
// **********************************************************************
class RCPropulsionModel : public PropulsionModelBase
{
public:
    // Constructor
    RCPropulsionModel(ModelMap *pMapInit, bool debugFlagIn = false);
 
    virtual void initialize(void);
private:
    typedef PropulsionModelBase Base;
    
    virtual void updatePropulsorStates(void);
    
    double throttle;
    double engineRPM;
    const double maxThrust = 3;
    const double maxRPM = 6440;
    const double location[3] = {0.3, 0.0, 0.0};
    const double orientation[3] = {0.0, 0.0, 0.0};
};

// **********************************************************************
// Quadcopter Propulsion Model
// **********************************************************************
class QuadcopterPropulsionModel : public PropulsionModelBase
{
public:
    // Constructor
    QuadcopterPropulsionModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    virtual void initialize(void);
private:
    typedef PropulsionModelBase Base;
    
    virtual void updatePropulsorStates(void);
    
    double throttle[4];
    double engineRPM[4];
    const double riseTimes[4] = {0.1, 0.1, 0.1, 0.1};
    const double maxThrust = 4.22; // 430g (12v) - 621g / 4.22N - 6.12N @12V
    const double maxRPM = 6440;
    const int directions[4] = {-1,1,-1,1};
    const double locations[4][3] ={
        {0.1, -0.1, 0.6},   // front left
        {0.1, 0.1, 0.6},    // front right
        {-0.1, 0.1, 0.6},   // rear right
        {-0.1, -0.1, 0.6}}; // rear left
    
    const double attitudes[4][3] = {
        {0.0, 90.0, 0.0},
        {0.0, 90.0, 0.0},
        {0.0, 90.0, 0.0},
        {0.0, 90.0, 0.0}};

};

// **********************************************************************
// Propulsion Type Base Class
// **********************************************************************
class PropulsionTypeBase : public GenericForceModel
{
public:
    // Constructor
    PropulsionTypeBase(ModelMap *pMapInit, bool debugFlagIn = false, double maxThrust_in=0.0, double maxRPM_in=0.0, int direction_in = 1);
    
    // Deconstructor
    PropulsionTypeBase(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    virtual void setLocation      (double* location_in)    { util.setUnitClassArray(location, location_in, meters, 3); }
    virtual void setOrientation   (double* orientation_in) { util.setUnitClassArray(orientation, orientation_in, degrees, 3); }
    virtual void setMaxThrust     (double maxThrust_in)    { maxThrust = maxThrust_in; }
    virtual void setMaxRPM        (double maxRPM_in)       { maxRPM = maxRPM_in; }
    virtual void setTorqueRatio   (double ratio_in)        { Q = ratio_in; }
    virtual void setThrottle      (double throttle_in)     { throttle = throttle_in; }
    virtual void setDirection     (int direction_in)      { direction = direction_in; }
    virtual void setEngineInertia (double *inertia_in)     { util.initMatrix(*engineInertia, *inertia_in, 3, 3); }
    
    virtual double getRPM (void) { return rpm; }
    
protected:
    class RotateFrame       *pRotate;
    class AtmosphereModel   *pAtmo;
    class DynamicsModel     *pDyn;
    
    DistanceType<double> location[3];
    AngleType<double>    orientation[3];
    double throttle;
    double maxThrust;
    double maxTorque;
    double maxRPM;
    double rpm;
    int direction; // positive is clockwise -> positive torque
    double engineInertia[3][3];
    
    double engineForce[3];
    double engineMoment[3];

    double Q; // thrust to torque ratio. Torque = thrust*Q (0.02 - 0.037)
    double b; // thrust coefficient thrust = b*omega^2
    double k; // torque coefficient torque = k*omega^2
    
    double qfEB[4]; // Quaternion from body frame to engine frame forces
    double qfBE[4]; // Quaternion from engine frame to body frame forces
    double qmEB[4]; // Quaternion from body frame to engine frame moments
    double qmBE[4]; // Quaternion from engine frame to body frame moments
    
    double REB[3][3]; // Rotation matrix from body frame to engine frame forces
    double RBE[3][3]; // Rotation matrix from engine frame to body frame forces
    double TEB[3][3]; // Rotation matrix from body frame to engine frame moments
    double TBE[3][3]; // Rotation matrix from engine frame to body frame moments
    
    void updateEngineRotations(void);
    virtual void calculateForcesAndMoments(void);
    void setBodyForcesAndMoments(void);
};

// **********************************************************************
// Simple Propeller Model
// **********************************************************************
class SimpleThrustEngine : public PropulsionTypeBase
{
public:
    // Constructor
    SimpleThrustEngine(ModelMap *pMapInit, bool debugFlagIn = false, double maxThrust_in=0.0, double maxRPM_in=0.0, int direction_in=1);
};

class Propeller : public PropulsionTypeBase
{
public:
    // Constructor
    Propeller(ModelMap *pMapInit, bool debugFlagIn=false, double maxThrust_in=0.0, double maxRPM_in=0.0, int direction_in=1);
    
    virtual void initialize(void);
    
    virtual void setMaxRPM(double rpm_in) { maxRPM = rpm_in; throttleController.setGain(maxRPM); }

    void setTimeConstant(double tau_in)   { throttleController.setTimeConstant(tau_in); }
private:
    typedef PropulsionTypeBase Base;
    
    virtual void calculateForcesAndMoments(void);
    
    FirstOrderFilter throttleController;
};
#endif /* propulsion_model_hpp */
