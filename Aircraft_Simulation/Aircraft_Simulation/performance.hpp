//
//  performance.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/31/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#ifndef performance_hpp
#define performance_hpp

#include <stdio.h>
#include "initial_conditions.hpp"
#include "generic_model.hpp"
#include "model_mapping.hpp"
#include "aircraft_simulation_types.h"

enum performanceForces {pThrust, pDrag, pLift, pWeight, nPerformanceForces};

class Performance : public GenericForceModel
{
public:
    // Constructor
    Performance(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    void LDmax(void);
    
    void trim(SpeedType<double> trimV = trimSpeed);
    
private:

    class DynamicsModel       *pDyn;
    class AeroModelBase       *pAero;
    class PropulsionModelBase *pProp;
    class AtmosphereModel     *pAtmo;
    class ActuatorModelBase   *pAct;
    class RotateFrame         *pRotate;
    class Time                *pTime;
    
    void updateConstants(void);
    void updateAeroCoeff(void);
    void trimModels(void);
    void updateForces(void);
    
    const static int nActuators = 4;
    
    // Coefficients
    double Cw, CT, CTmax, LDratio;
    double Cd, Cdo;
    double CL, CLo, CLa, CLq, CLde;
    double Cm, Cmo, Cma, Cmq, Cmde;
    double CY, CYb, CYp, CYr, CYda, CYdr;
    double Cl, Clb, Clp, Clr, Clda, Cldr;
    double Cn, Cnb, Cnp, Cnr, Cnda, Cndr;
    
    // rates
    double qhat;
    
    // Atmosphere
    double* aeroMatrix;
    double qdyn;
    double qS; // qdyn*wingArea
    
    // Matrix solving
    double lonConsts[2];    // { {Cw - CLo}, {-Cmo} }
    double lonMatrix[2][2]; // { {CLa CLde}, {Cma Cmde} }
    double lonTrimSoln[2];  // {alpha, de}
    
    double latConsts[2];    //
    double latMatrix[3][3]; // { {CYb, CYda, CYdr}, {Clb, Clda, Cldr}, {Cnb, Cnda, Cndr} }
    double latTrimSoln[3];  // {beta, da, dr}
    
    // Forces and Moments
    double forces[nPerformanceForces]; // Thrust, Drag, Lift, Weight
    double aeroFroces[3]; // Drag, Sideforce, Lift
    
    // Trim
    AngleType<double> trimAttitude[3]
    {
        AngleType<double>(0,degrees), // roll
        AngleType<double>(0,degrees), // pitch
        AngleType<double>(0,degrees)  // yaw
    };
    
    AngleType<double> trimAero[3]
    {
        AngleType<double>(0,degrees), // aerodynamic roll angle
        AngleType<double>(0,degrees), // angle of attack
        AngleType<double>(0,degrees)  // sideslip
    };
    
    double controls[nActuators]; // de, da, dr, dT
    
    SpeedType<double> trimV { SpeedType<double>(0,metersPerSecond) };
};
#endif /* performance_hpp */
