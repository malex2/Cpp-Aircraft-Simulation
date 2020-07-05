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

enum airValues {density, pressure, dynPress, temp, dynVisc, specificHeat, R};

class AtmosphereModel : public GenericForceModel
{
public:
    // Constructor
    AtmosphereModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    SpeedType<float>* getVelWindBody(void) { return velWindBody; }
    SpeedType<float>* getVelWindNED(void) { return velWindNED; }
    float* getAir(void) { return air; }
    
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    
    // Functions
    void updateAir(void);
    
    void updateGravity(void);
    
    void updateWind(void);
    
    SpeedType<float> velWindNED[3];
    SpeedType<float> velWindBody[3];
    
    float nedForce[3];
    
    float air[7]; // density, pressure, dynamic pressure, temperature, dynamic viscocity, R;
    float Re;   // Reynolds number
    float Mach; // Mach number
    SpeedType<float> speedOfSound;
    
    // print variables
    float gravity;
};

#endif /* Atmosphere_hpp */
