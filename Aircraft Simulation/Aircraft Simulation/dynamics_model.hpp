//
//  DynamicsModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/22/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef DynamicsModel_hpp
#define DynamicsModel_hpp

#include "generic_model.hpp"

class DynamicsModel : public GenericForceModel
{
public:
    
    // Constructor
    DynamicsModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update dynamic states
    virtual bool update(void);
    
    // Getters
    double*               getPosBody(void)        { return posBody; }   // Lat, Lon, Alt
    DistanceType<float>*  getPosRelNED(void)      { return posRelNED; } // N, E, D from start in m
    DistanceType<double>  gethCenter(void)        { return hCenter; }   // Distance above Earth center
    DistanceType<float>   gethGround(void)        { return hGround; }   // Distance above ground
    
    SpeedType<float>*     getVelBody(void)        { return velBody; }
    SpeedType<float>*     getVelBodyRelWind(void) { return velBodyRelWind; }
    SpeedType<float>*     getVelNED(void)         { return velNED; }
    SpeedType<float>*     getVelNEDRelWind(void)  { return velNEDRelWind; }
    SpeedType<float>      getSpeed(void)          { return gndVel; }
    
    float*                getAccBody(void)        { return accBody; }
    float*                getbodyAngularAcc(void) { return bodyAngularAcc; }
    
    AngleType<float>*     getEulerAngles(void)    { return eulerAngles; }
    
    AngleRateType<float>* getEulerRates(void)     { return eulerRates; }
    AngleRateType<float>* getBodyRates(void)      { return bodyRates; }
    
    float*                get_q_B_NED(void)       { return q_B_NED; }
    float*                get_q_B_LL(void)        { return q_B_LL; }
    
    float                 getMass(void)           { return mass; }
    
private:
    
    // Classes
    class RotateFrame     *pRotate;
    class AeroModel       *pAero;
    class PropulsionModel *pProp;
    class AtmosphereModel *pAtmo;
    class GroundModel     *pGnd;
    class Time            *pTime;
    
    double posBody[3];                 // Lat (rad), Lon (rad), Alt (m)
    float posBodyPrint[3];             // Lat (deg), Lon (deg), Alt (ft)
    DistanceType<float> posRelNED[3];  // N, E, D distance from start
    DistanceType<double> hCenter;      // Distance of CG above Earth center
    DistanceType<float> hGround;       // Distance of CG above ground
    DistanceType<float> elevation;     // Height above mean sea level (MSL) of ground
    
    //  --------------------- Aircraft            ---     ---
    //            |                                |       |
    //          hGround                            |       |
    //            |                                |       |
    //  --------------------- Ground            Altitude   |
    //            |                                |       |
    //        Elevation                            |     hCenter
    //            |                                |       |
    //  --------------------- Mean Sea Level      ---      |
    //            |                                        |
    //          Rearth                                     |
    //            |                                        |
    //  --------------------- Earth Center                ---
    
    SpeedType<float> velBody[3];
    SpeedType<float> velBodyRelWind[3];
    SpeedType<float> velNED[3];
    SpeedType<float> velNEDRelWind[3];
    SpeedType<float> gndVel;
    
    double dPosBody[3]; // Lat rate, Lon rate, Alt rate
    float accBody[3];
    
    AngleType<float> eulerAngles[3];
    float q_B_NED[4];
    float q_B_LL[4];
    float q_B_NED_dot[4];
    
    AngleRateType<float> bodyRates[3];
    AngleRateType<float> eulerRates[3];
    float bodyAngularAcc[3];
    
    float inertia[3][3];
    float mass;
    
    // print variables
    AngleType<float> eulerAnglesDeg[3];
    AngleType<float> eulerCheckDeg[3];
    AngleRateType<float> bodyRatesDeg[3];
    AngleRateType<float> eulerRatesDeg[3];
    float hGroundft;
};

#endif /* DynamicsModel_hpp */
