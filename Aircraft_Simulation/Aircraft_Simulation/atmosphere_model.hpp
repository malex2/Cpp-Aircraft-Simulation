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
    enum atmosphereType {TROPOSPHERE, STRATOSPHERE1, STRATOSPHERE2, STRATOSPHERE3, MESOSPHERE1, MESOSPHERE2, THERMOSPHERE1, THERMOSPHERE2, nATMOSPHERELAYERS};
    enum temperatureGradientType {CONSTANT, SLOPE};
    
    struct layerInfoType {
        atmosphereType layer;
        temperatureGradientType gradient;
        double basePressure;
        double baseAltitude;
        double baseTemperature;
        double baseDensity;
        double dTdH;
        
        layerInfoType()
        {
            layer = TROPOSPHERE;
            gradient = SLOPE;
            baseAltitude = 0.0;
            baseDensity  = 1.225;
            basePressure = 101325.0;
            baseTemperature  = 288.16;
            dTdH = 0.0;
        }
    };
    
    layerInfoType layerInfo;
    
    // Getters
    double* getVelWindBody(void)  { return velWindBody; }
    double* getVelWindNED(void)   { return velWindNED; }
    double  getGravity(void)      { return gravity; }
    double* getGravityNED(void)   { return nedGravity; }
    double* getGravityBody(void)  { return bodyGravity; }
    double* getGravityEuler(void) { return gravityEuler; }
    double* getAir(void)          { return air; }
    double  getRe(void)           { return Re;  }
    double  getMach(void)         { return Mach; }
    double  getGravityRoll(void)  { return gravity_roll; }
    double  getGravityPitch(void) { return gravity_pitch; }
private:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    
    // Functions
    void updateAir(void);
    
    void updateGravity(void);
    
    void updateWind(void);
    
    void updateLayerInfo(double altitude);
    
    double constantPressure(double basePressure, double baseAltitude, double altitude, double temperature);
    double constantDensity(double baseDensity, double baseAltitude, double altitude, double temperature);
    double gradientPressure(double basePressure, double baseTemperature, double temperature, double dTdH);
    double gradientDensity(double baseDensity, double baseTemperature, double temperatre, double dTdH);
    double gradientTemperature(double baseTemperature, double baseAltitude, double altitude, double dTdH);
    
    double velWindNED[3];
    double velWindBody[3];
    
    double nedForce[3];
    double bodyGravity[3];
    double nedGravity[3];
    double gravityEuler[3];
    double air[7]; // density, pressure, dynamic pressure, temperature, dynamic viscocity, R;
    double Re;   // Reynolds number
    double Mach; // Mach number
    SpeedType<double> speedOfSound;
    
    double ALTITUDE[nATMOSPHERELAYERS];
    double TEMPERATURE[nATMOSPHERELAYERS];
    double PRESSURE[nATMOSPHERELAYERS];
    double DENSITY[nATMOSPHERELAYERS];
    double DTDH[nATMOSPHERELAYERS];
    temperatureGradientType GRADIENT[nATMOSPHERELAYERS];
    
    // Atmospheric References
    const double gravitySL = 9.81;
    const double spaceTempInShadow = 170.0;
    const double spaceTempInSun    = 393.0;
    const double spaceTempDistant  = 3.0;
    
    // print variables
    double gravity;
    double layer;
    double pressureMB;
    double tempC;
    double gravity_roll;
    double gravity_pitch;
    
    double gravity_roll_deg;
    double gravity_pitch_deg;
};

#endif /* Atmosphere_hpp */
