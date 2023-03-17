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

// **********************************************************************
// Contact Point Model
// Part of aircraft that can exert force when touching ground
// **********************************************************************
class ContactPoint : public GenericForceModel
{
public:
    // Constructor
    ContactPoint(ModelMap *pMapInit, double x, double y, double z, double kIn, double zetaIn, double muxStaticIn, double muyStaticIn, double muxDynamicIn, double muyDynamicIn, bool debugFlagIn = false);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    bool                  isOnGround(void) { return onGround; };
    double*               getLLForce(void) { return LLForce;  };
    DistanceType<double>* getPos(void)     { return posRelCG; };
    
    // Setters
    void setPercentLoad (double percLoadIn) { percLoad = percLoadIn; }
    
    // Keep track of maximum friction in each direction
    static int iContactPoint;  // current contact point
    static int nContactPoints;
private:
    class DynamicsModel   *pDyn;
    class RotateFrame     *pRotate;
    class GroundModelBase *pGround;
    
    // Internal Functions
    void   groundToBody(double* vBody, double* vGround);
    void   bodyToGround(double* vGround, double* vBody);
    void   bodyToGround(SpeedType<double> * vGround, SpeedType<double> * vBody);
    void   sumExternalForces(void);
    double determineFriction(double maxFriction, double externalForce, double velocity, double muDynamic);
    
    // States relative to CG in Body frame
    DistanceType<double> posRelCG[3];
    SpeedType<double>    velRelCG[3];
    
    // States relative to CG in Local Level frame
    DistanceType<double> posLL[3];
    DistanceType<double> posRelNED[3];
    SpeedType<double>    velLL[3];
    
    // Height of object above ground
    DistanceType<double> posNED[3];
    DistanceType<double> altGround;
    double               groundElevation;
    double               hMSL_init;
    
    // Dynamics constants
    double k;           // spring constant
    double b;           // damping constant
    double zeta;        // damping ratio
    double muxStatic;   // x coefficient of static friction
    double muxDynamic;  // x coefficient of dynamic friction
    double muyStatic;   // y coefficient of static friction
    double muyDynamic;  // y coefficient of dynamic friction
    
    // External forces
    double bodyForceExt[3];
    double LLForceExt[3];
    
    bool   onGround;
    double percLoad; // percentage of total load this contact point holds
};

// **********************************************************************
// Ground Model Base
// **********************************************************************
class GroundModelBase : public GenericForceModel
{
public:
    // Constructor
    GroundModelBase(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Destructor
    ~GroundModelBase(void);
    
    // Initialize class pointers and variables that rely on other classes
    virtual void initialize(void);
    
    // Update obejct states and forces
    virtual bool update(void);
    
    // Getters
    virtual bool isOnGround(void) { return util.any(onGround, nContactPoints); }
    
    double getGroundElevation(double north, double east);
    void   getGroundAngles(double north, double east, double* ground_euler);
protected:
    class DynamicsModel *pDyn;
    class RotateFrame   *pRotate;
    class Time          *pTime;
    
    // Number of contact points
    int nContactPoints;
    const static int maxContactPoints = 10;
    ContactPoint *contactPoints[maxContactPoints];
    
    bool onGround[maxContactPoints];
    
    double onGroundPrint;
    double groundHeight;
    double groundEuler[3];
    
    // Functions
    void setLoadPercentage();
    
    // Ground Grid (Above MSL)
    static const int N_NORTH_GRID_POINTS = 11;
    static const int N_EAST_GRID_POINTS  = 11;
    const double NORTH_ELEVATION_POINTS[N_NORTH_GRID_POINTS] = {-100.0, -80.0, -60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0};
    const double EAST_ELEVATION_POINTS[N_EAST_GRID_POINTS]   = {-100.0, -80.0, -60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0};
    
    #ifdef STEEP_GROUND
    const double GRID_ELEVATION[N_NORTH_GRID_POINTS][N_EAST_GRID_POINTS] = {
//East:-100.0, -80.0, -60.0, -40.0, -20.0, 0.0  , 20.0 , 40.0 , 60.0 , 80.0 , 100.0     North
    { -25.0  , -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0}, // -100.0
    { -20.0  , -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0}, // -80.0
    { -15.0  , -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0}, // -60.0
    { -10.0  , -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0}, // -40.0
    { -5.0   , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 , -5.0 }, // -20.0
    { 0.0    , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  }, // 0.0
    { 5.0    , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  , 5.0  }, // 20.0
    { 10.0   , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 }, // 40.0
    { 15.0   , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 }, // 60.0
    { 20.0   , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 }, // 80.0
    { 25.0   , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 }  // 100.0
    };
    
  #elif SHALLOW_GROUND
    const double GRID_ELEVATION[N_NORTH_GRID_POINTS][N_EAST_GRID_POINTS] = {
        //East:-100.0, -80.0, -60.0, -40.0, -20.0, 0.0  , 20.0 , 40.0 , 60.0 , 80.0 , 100.0     North
        { -25.0  , -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0}, // -100.0
        { -20.0  , -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0}, // -80.0
        { -15.0  , -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0}, // -60.0
        { -10.0  , -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0}, // -40.0
        { -5.0   , -5.0 , -5.0 , -5.0 , -1.0 , -1.0 , -1.0 , -5.0 , -5.0 , -5.0 , -5.0 }, // -20.0
        { 0.0    , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  }, // 0.0
        { 5.0    , 5.0  , 5.0  , 5.0  , 1.0  , 1.0  , 1.0  , 5.0  , 5.0  , 5.0  , 5.0  }, // 20.0
        { 10.0   , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 }, // 40.0
        { 15.0   , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 }, // 60.0
        { 20.0   , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 }, // 80.0
        { 25.0   , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 }  // 100.0
    };
    
    #else // FLAT_GROUND
    const double GRID_ELEVATION[N_NORTH_GRID_POINTS][N_EAST_GRID_POINTS] = {
        //East:-100.0, -80.0, -60.0, -40.0, -20.0, 0.0  , 20.0 , 40.0 , 60.0 , 80.0 , 100.0     North
        { -25.0  , -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0}, // -100.0
        { -20.0  , -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -20.0}, // -80.0
        { -15.0  , -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0}, // -60.0
        { -10.0  , -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0}, // -40.0
        { -5.0   , -5.0 , -5.0 , -5.0 , 0.0 , 0.0 , 0.0 , -5.0 , -5.0 , -5.0 , -5.0 }, // -20.0
        { 0.0    , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  , 0.0  }, // 0.0
        { 5.0    , 5.0  , 5.0  , 5.0  , 0.0  , 0.0  , 0.0  , 5.0  , 5.0  , 5.0  , 5.0  }, // 20.0
        { 10.0   , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 , 10.0 }, // 40.0
        { 15.0   , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 , 15.0 }, // 60.0
        { 20.0   , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 , 20.0 }, // 80.0
        { 25.0   , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 , 25.0 }  // 100.0
    };
     #endif
  
};

// **********************************************************************
// RC Plance Ground Model
// **********************************************************************
class RCGroundModel : public GroundModelBase
{
public:
    // Constructor
    RCGroundModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Destructor
    RCGroundModel(void);
private:
    /*
     const double cgz = 0.09; // Height of center of gravity above ground when aircraft is on ground (0.3 ft)
     const double Lg1 = 0.1; // Length of front landing gear
     const double Lg2 = 0.06; // Length of rear landing gear
     const double L1 = 0.06; // Distance of front landing gear from CG
     const double L2 = 0.29; // Distance of rear landing gear from CG
     const double LL = 0.05; // Distance of left landing gear from CG
     const double LR = 0.05; // Distance of right landing gear from CG
     const double thetag = asin((Lg1-Lg2)/(L1+L2))*rad2deg; // Pitch of aircraft on flat ground = 6.56 deg
     */
    
    // Left Front Landing Gear
    const double k1L    = 10000;
    const double zeta1L = 1;
    const double mux1L_static = 0.08; // 0.02-0.08
    const double mux1L_dyn    = 0.04;
    const double muy1L_static = 0.5;
    const double muy1L_dyn    = 0.3;
    const double pos1L[3]     = {0.06, -0.05, 0.1};
    
    // Right Front Landing Gear
    const double k1R    = 10000;
    const double zeta1R = 1;
    const double mux1R_static = 0.08;
    const double mux1R_dyn    = 0.04;
    const double muy1R_static = 0.5;
    const double muy1R_dyn    = 0.3;
    const double pos1R[3]     = {0.06, 0.05, 0.1};
    
    // Rear Landing Gear
    const double k2     = 10000;
    const double zeta2  = 1;
    const double mux2_static = 0.08;
    const double mux2_dyn    = 0.08;
    const double muy2_static = 0.5;
    const double muy2_dyn    = 1.08;
    const double pos2[3]     = {-0.29, 0, 0.06};
};

// **********************************************************************
// Quadcopter Ground Model
// **********************************************************************
class QuadcopterGroundModel : public GroundModelBase
{
public:
    // Constructor
    QuadcopterGroundModel(ModelMap *pMapInit, bool debugFlagIn = false);
    
    // Destructor
    QuadcopterGroundModel(void);
private:
    // Left Front Landing Gear
    const double k1    = 1000;
    const double zeta1 = 1;
    const double mux1_static = 0.32; // 0.02-0.08
    const double mux1_dyn    = 0.16;
    const double muy1_static = 0.32;
    const double muy1_dyn    = 0.16;
    const double pos1[3]     = {0.1, -0.1, 0.1};
    
    // Right Front Landing Gear
    const double k2    = 1000;
    const double zeta2 = 1;
    const double mux2_static = 0.32;
    const double mux2_dyn    = 0.16;
    const double muy2_static = 0.32;
    const double muy2_dyn    = 0.16;
    const double pos2[3]     = {0.1, 0.1, 0.1};
    
    // Right Rear Landing Gear
    const double k3     = 1000;
    const double zeta3  = 1;
    const double mux3_static = 0.32;
    const double mux3_dyn    = 0.16;
    const double muy3_static = 0.32;
    const double muy3_dyn    = 0.16;
    const double pos3[3]     = {-0.1, 0.1, 0.1};
    
    // Left Rear Landing Gear
    const double k4     = 1000;
    const double zeta4  = 1;
    const double mux4_static = 0.32;
    const double mux4_dyn    = 0.16;
    const double muy4_static = 0.32;
    const double muy4_dyn    = 0.16;
    const double pos4[3]     = {-0.1, -0.1, 0.1};
};

#endif /* GroundModel_hpp */
