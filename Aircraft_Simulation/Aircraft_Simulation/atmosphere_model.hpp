//
//  Atmosphere.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/28/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef AtmosphericModel_hpp
#define AtmosphericModel_hpp

#include "generic_model.hpp"

class AtmosphereModel : public GenericForceModel
{
public:
    // Constructor
    AtmosphereModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Enumeration
    enum airValues {density, pressure, dynPress, temp, dynVisc, specificHeat, R};
    
    // Getters
    SpeedType<double>* getVelWindBody(void) { return velWindBody; }
    SpeedType<double>* getVelWindNED(void)  { return velWindNED; }
    double             getGravity(void)     { return gravity; }
    double*            getAir(void)         { return air; }
    double             getRe(void)          { return Re;  }
    double             getMach(void)        { return Mach; }
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    
    // Functions
    void updateAir(void);
    
    void updateGravity(void);
    
    void updateWind(void);
    
    SpeedType<double> velWindNED[3];
    SpeedType<double> velWindBody[3];
    
    double nedForce[3];
    
    double air[7]; // density, pressure, dynamic pressure, temperature, dynamic viscocity, R;
    double Re;   // Reynolds number
    double Mach; // Mach number
    SpeedType<double> speedOfSound;
    
    // print variables
    double gravity;
};

#endif /* Atmosphere_hpp */
