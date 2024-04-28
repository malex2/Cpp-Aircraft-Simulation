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
    
    void deltaIMU(double dt);
    void resetIMU();
    
    // Getters
    double* getPosLLH(void)   { return posLLH; }   // Lat, Lon, Alt
    double* getPosECEF(void)  { return posECEF;}  // ECEF Position
    double* getPosNED(void)   { return posNED; } // N, E, D from start in m
    double* getPosInit(void)  { return posInit; }
    double  getAltMSL(void)   { return hMSL;   }
    double  gethCenter(void)  { return hCenter; }   // Distance above Earth center
    double  gethGround(void)  { return hGround; }   // Distance above ground
    
    double* getVelBody(void)  { return velBody; }
    double* getVelNED(void)   { return velNED; }
    double* getVelLL(void)    { return velLL; }
    double* getVelInit(void)  { return velInit; }
    
    double  getSpeed(void)    { return velMag; }
    
    double* getAccBody(void)        { return accelBody; }
    double* getAccLL(void)          { return accelLL; }
    double* getbodyAngularAcc(void) { return bodyAngularAcc; }
    
    double* getEulerAngles(void)    { return eulerAngles; }
    double  getBearing(void)        { return bearing; }
    double* getBodyRates(void)      { return bodyRates; }
    
    double* get_q_B_NED(void)       { return q_B_NED; }
    double  getMass(void)           { return mass; }
    double  getTimestamp(void)      { return timestamp; }

    double* getDeltaTheta(void)       { return deltaTheta; }
    double* getDeltaVelocity(void)    { return deltaVelocity; }
    double* getDeltaVelocityNED(void) { return deltaVelocityNED; }
    double* getDeltaPositionNED(void) { return deltaPositionNED; }
private:
    // Classes
    class RotateFrame         *pRotate;
    class AtmosphereModel     *pAtmo;
    class GroundModelBase     *pGround;
    class Time                *pTime;
    
    // Functions
    void sumForces();
    void updatePosition();
    void updateAttitude();
    void updateStates();
    double getEllipsoidHeight();
    
    // Variables
    double deltaTheta[3];
    double deltaVelocity[3];
    double deltaVelocityNED[3];
    double prevVelNED[3];
    double deltaPositionNED[3];
    double prevPosNED[3];
    
    double forceECI[3];
    double accelECI[3];
    double accelBody[3];
    double accelLL[3];
    double accelMag;
    
    double velECI[3];
    double velECEF[3];
    double velNED[3];
    double velBody[3];
    double velLL[3];
    double velInit[3];
    double velMag;
    
    double posECI[3];
    double posLLH[3];
    double posECEF[3];
    double posECEF_init[3];
    double d_posECEF[3];
    double posNED[3];
    double posInit[3];
    double posAboveOrigin;
    
    double hCenter;
    double hGround;
    double hMSL;
    double groundElevation;
    
    double bodyAngularAcc[3];
    double bodyRates[3];
    
    double q_B_NED[4];
    double dq_B_NED[4];
    double q_B_NED_dot[4];
    double eulerAngles[3];
    double bearing;
    
    double inertia[3][3];
    double mass;
    double timestamp;
    
    // print variables
    double posLLH_deg[3];
    double bodyRatesDeg[3];
    double eulerAnglesDeg[3];
    double deltaThetaDeg[3];
    double bearingDeg;
    
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
};

#endif /* DynamicsModel_hpp */
