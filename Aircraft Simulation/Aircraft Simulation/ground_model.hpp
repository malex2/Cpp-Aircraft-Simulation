//
//  GroundModel.hpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/4/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#ifndef GroundModel_hpp
#define GroundModel_hpp

#include "generic_model.hpp"
#include "utilities.hpp"

// Part of aircraft that can exert force when touching ground
class ContactPoint : public GenericForceModel
{
public:
    // Constructor
    ContactPoint(ModelMap *pMapInit, float x, float y, float z, float kIn, float zetaIn, float muxStaticIn, float muyStaticIn, float muxDynamicIn, float muyDynamicIn, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    bool                 isOnGround(void) { return onGround; };
    float*               getLLForce(void) { return LLForce;  };
    DistanceType<float>* getPos(void)     { return posRelCG; };
    
    // Setters
    void setPercentLoad (float percLoadIn) { percLoad = percLoadIn; }
    
private:
    class DynamicsModel   *pDyn;
    class RotateFrame     *pRotate;
    
    // Internal Functions
    void sumExternalForces(void);
    float determineFriction(float maxFriction, float externalForce, float velocity, float muDynamic);
    
    // States relative to CG in Body frame
    DistanceType<float> posRelCG[3];
    SpeedType<float>    velRelCG[3];
    
    // States relative to CG in Local Level frame
    DistanceType<float> posLL[3];
    SpeedType<float>    velLL[3];
    
    // Height of object above ground
    DistanceType<float> altGround;
    
    // Dynamics constants
    float k;           // spring constant
    float b;           // damping constant
    float zeta;        // damping ratio
    float muxStatic;   // x coefficient of static friction
    float muxDynamic;  // x coefficient of dynamic friction
    float muyStatic;   // y coefficient of static friction
    float muyDynamic;  // y coefficient of dynamic friction
    
    // Keep track of maximum friction in each direction
    static int iContactPoint;  // current contact point
    static int nContactPoints; // total number of contact points
    
    // External forces
    float bodyForceExt[3];
    float LLForceExt[3];
    
    bool onGround;
    float percLoad; // percentage of total load this contact point holds
};

class GroundModel : public GenericForceModel
{
public:
    // Constructor
    GroundModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Destructor
    ~GroundModel(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
private:
    /*
    const float cgz = 0.09; // Height of center of gravity above ground when aircraft is on ground (0.3 ft)
    const float Lg1 = 0.1; // Length of front landing gear
    const float Lg2 = 0.06; // Length of rear landing gear
    const float L1 = 0.06; // Distance of front landing gear from CG
    const float L2 = 0.29; // Distance of rear landing gear from CG
    const float LL = 0.05; // Distance of left landing gear from CG
    const float LR = 0.05; // Distance of right landing gear from CG
    const float thetag = asin((Lg1-Lg2)/(L1+L2))*rad2deg; // Pitch of aircraft on flat ground = 6.56 deg
    */
    
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    class Time          *pTime;
    
    // Number of contact points
    static const int nContactPoints = 3;
    ContactPoint *contactPoints[nContactPoints];
    
    bool onGround[nContactPoints];
    
    // Functions
    void setLoadPercentage(void);
    
    // Left Front Landing Gear
    const float k1L    = 10000;
    const float zeta1L = 1;
    const float mux1L_static = 0.08; // 0.02-0.08
    const float mux1L_dyn    = 0.04;
    const float muy1L_static = 0.5;
    const float muy1L_dyn    = 0.3;
    const float pos1L[3]     = {0.06, -0.05, 0.1};
    
    // Right Front Landing Gear
    const float k1R    = 10000;
    const float zeta1R = 1;
    const float mux1R_static = 0.08;
    const float mux1R_dyn    = 0.04;
    const float muy1R_static = 0.5;
    const float muy1R_dyn    = 0.3;
    const float pos1R[3]     = {0.06, 0.05, 0.1};
    
    // Rear Landing Gear
    const float k2     = 10000;
    const float zeta2  = 1;
    const float mux2_static = 0.08;
    const float mux2_dyn    = 0.08;
    const float muy2_static = 0.5;
    const float muy2_dyn    = 1.08;
    const float pos2[3]     = {-0.29, 0, 0.06};
};

#endif /* GroundModel_hpp */
