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
    DistanceType<double>*  getPosRelNED(void)      { return posRelNED; } // N, E, D from start in m
    DistanceType<double>  gethCenter(void)        { return hCenter; }   // Distance above Earth center
    DistanceType<double>   gethGround(void)        { return hGround; }   // Distance above ground
    
    SpeedType<double>*     getVelBody(void)        { return velBody; }
    SpeedType<double>*     getVelBodyRelWind(void) { return velBodyRelWind; }
    SpeedType<double>*     getVelNED(void)         { return velNED; }
    SpeedType<double>*     getVelNEDRelWind(void)  { return velNEDRelWind; }
    SpeedType<double>      getSpeed(void)          { return gndVel; }
    
    double*                getAccBody(void)        { return accBody; }
    double*                getbodyAngularAcc(void) { return bodyAngularAcc; }
    
    AngleType<double>*     getEulerAngles(void)    { return eulerAngles; }
    
    AngleRateType<double>* getEulerRates(void)     { return eulerRates; }
    AngleRateType<double>* getBodyRates(void)      { return bodyRates; }
    
    double*                get_q_B_NED(void)       { return q_B_NED; }
    double*                get_q_B_LL(void)        { return q_B_LL; }
    
    double                 getMass(void)           { return mass; }
    
    // Setters
    void setEulerAngles(AngleType<double>* angles_in);
    
    void setSpeed(SpeedType<double> vel_in);
    
private:
    
    // Classes
    class RotateFrame         *pRotate;
    class AtmosphereModel     *pAtmo;
    class Time                *pTime;
    
    double posBody[3];                 // Lat (rad), Lon (rad), Alt (m)
    double posBodyPrint[3];             // Lat (deg), Lon (deg), Alt (ft)
    DistanceType<double> posRelNED[3];  // N, E, D distance from start
    DistanceType<double> hCenter;      // Distance of CG above Earth center
    DistanceType<double> hGround;       // Distance of CG above ground
    DistanceType<double> elevation;     // Height above mean sea level (MSL) of ground
    
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
    
    SpeedType<double> velBody[3];
    SpeedType<double> velBodyRelWind[3];
    SpeedType<double> velNED[3];
    SpeedType<double> velNEDRelWind[3];
    SpeedType<double> gndVel;
    
    double dPosBody[3]; // Lat rate, Lon rate, Alt rate
    double accBody[3];
    
    AngleType<double> eulerAngles[3];
    double q_B_NED[4];
    double q_B_LL[4];
    double q_B_NED_dot[4];
    
    AngleRateType<double> bodyRates[3];
    AngleRateType<double> eulerRates[3];
    double bodyAngularAcc[3];
    
    double inertia[3][3];
    double mass;
    
    // print variables
    AngleType<double> eulerAnglesDeg[3];
    AngleType<double> eulerCheckDeg[3];
    AngleRateType<double> bodyRatesDeg[3];
    AngleRateType<double> eulerRatesDeg[3];
    double hGroundft;
};

#endif /* DynamicsModel_hpp */
