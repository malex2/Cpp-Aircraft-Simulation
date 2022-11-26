//
//  GroundModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 8/4/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "initial_conditions.hpp"
#include "rotate_frame.hpp"
#include "dynamics_model.hpp"
#include "model_mapping.hpp"
#include "time.hpp"

#include "ground_model.hpp"

// **********************************************************************
// Contact Point Model
// **********************************************************************
int ContactPoint::iContactPoint  = 0;
int ContactPoint::nContactPoints = 0;
ContactPoint::ContactPoint(ModelMap *pMapInit, double x, double y, double z, double kIn, double zetaIn, double muxStaticIn, double muyStaticIn, double muxDynamicIn, double muyDynamicIn, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pGround = NULL;
    
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    // position relative to CG
    double posRelCG_in[3];
    posRelCG_in[0] = x;
    posRelCG_in[1] = y;
    posRelCG_in[2] = z;
    util.setUnitClassArray(posRelCG, posRelCG_in, meters, 3);
    
    // Velocity relative to CG
    util.setUnitClassArray(velRelCG, zero_init, metersPerSecond, 3);
    
    // Height above ground
    altGround.convertUnit(meters);
    hMSL_init = posLLH_init[2]*util.ft2m;
    
    // Spring constants
    k    = kIn;
    zeta = zetaIn;
    
    // Friction
    muxStatic  = muxStaticIn;
    muxDynamic = muxDynamicIn;
    
    muyStatic  = muyStaticIn;
    muyDynamic = muyDynamicIn;
    
    nContactPoints++;
}

void ContactPoint::initialize(void)
{
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pGround = (GroundModelBase*) pMap->getModel("GroundModel");
    
    percLoad = 1.0;
    
    // Damping
    b  = 2.0*zeta*sqrt(k*pDyn->getMass());
}

bool ContactPoint::update(void)
{
    iContactPoint++;
    
    SpeedType<double> velRel[3];
    SpeedType<double> velBody[3];
    SpeedType<double> cgVel_B[3];
    AngleRateType<double> bodyRates[3];
    
    // Update position relative to ground
    //pRotate->bodyToLL(posLL, posRelCG);
    pRotate->bodyToNED(posRelNED, posRelCG);
    
    util.vAdd(posNED, posRelNED, pDyn->getPosNED(), 3);
    posNED[2].val -= hMSL_init;
    
    groundElevation = pGround->getGroundElevation(posNED[0].m(), posNED[1].m());

    altGround.val = -posNED[2].m() - groundElevation;
    
    // Update Local Level velocity
    util.setUnitClassArray(velRel, zero_init, metersPerSecond, 3);
    util.setUnitClassArray(bodyRates, pDyn->getBodyRates(), radiansPerSecond, 3);
    util.crossProduct(velRel, bodyRates, posRelCG);
    
    util.setUnitClassArray(cgVel_B, pDyn->getVelBody(), metersPerSecond, 3);
    util.vAdd(velBody, cgVel_B, velRel, 3);
    
    bodyToGround(velLL, velBody);
    
    if (altGround.val <= zeroTolerance ) { onGround = true; }
    else { onGround = false; }
    
    if (onGround)
    {
        double m = pDyn->getMass();
        b  = 2*zeta*sqrt(k*m);
        
        // Update normal force (Local Level force in Z direction)
        LLForce[2] = k*altGround.val - b*velLL[2].val;
        
        // Normal force always up (negative Local Level z direction) or zero
        if (LLForce[2] > 0) { LLForce[2] = 0; }
        
        // Get aerodynamic, atmospheric, and propulsion forces in local level frame
        sumExternalForces();
        
        // Update friction
        double FxMax = muxStatic*LLForce[2];
        LLForce[0] = determineFriction(FxMax, LLForceExt[0]*percLoad, velLL[0].val, muxStatic);
        
        double FyMax = muyStatic*LLForce[2]*percLoad;
        LLForce[1] = determineFriction(FyMax, LLForceExt[1]*percLoad, velLL[1].val, muyDynamic);
    }
    else
    {
        util.setArray(LLForce, zero_init, 3);
    }
    groundToBody(bodyForce, LLForce);
    //pRotate->LLToBody(bodyForce, LLForce);
    
    // Update moments
    util.crossProduct(bodyMoment, posRelCG, bodyForce);
    
    if (debugFlag)
    {
        printf("Contact Point %d Update: \n", iContactPoint);
        util.print(posRelCG, 3, "posRelCG:");
        util.print(posLL, 3, "posLL:");
        util.print(velBody, 3, "point velocity body frame:");
        util.print(velLL, 3, "point velocity LL:");
        util.print(&altGround, 1, "altGround:");
        util.print(LLForce, 3, "Ground Force LL ");
        util.print(bodyForce, 3, "Ground Force Body ");
        util.print(bodyMoment, 3, "Ground Body Moment");
    }
    
    if (iContactPoint == nContactPoints) { iContactPoint = 0; }
    
    return true;
}

void ContactPoint::groundToBody(double* vBody, double* vGround)
{
    double euler_NED2Ground[3] = {0.0, 0.0, 0.0};
    double R_NED2Ground[3][3];
    double R_Ground2NED[3][3];
    double vNED[3];
    
    // Get rotation from ground frame to NED frame
    pGround->getGroundAngles(posNED[0].val, posNED[1].val, euler_NED2Ground);
    util.setupRotation(*R_NED2Ground, euler_NED2Ground);
    util.mtran(*R_Ground2NED, *R_NED2Ground, 3, 3);
    
    // Rotate Ground -> NED, NED -> Body
    util.mmult(vNED, *R_Ground2NED, vGround, 3, 3);
    pRotate->NEDToBody(vBody, vNED);
}

void ContactPoint::bodyToGround(double* vGround, double* vBody)
{
    double euler_NED2Ground[3] = {0.0, 0.0, 0.0};
    double R_NED2Ground[3][3];
    double vNED[3];
    
    // Get rotation from NED frame to ground frame
    pGround->getGroundAngles(posNED[0].val, posNED[1].val, euler_NED2Ground);
    util.setupRotation(*R_NED2Ground, euler_NED2Ground);
    
    // Rotate Body -> NED, NED -> Ground
    pRotate->bodyToNED(vNED, vBody);
    util.mmult(vGround, *R_NED2Ground, vNED, 3, 3);
}


void ContactPoint::bodyToGround(SpeedType<double> * vGround, SpeedType<double> * vBody)
{
    double euler_NED2Ground[3] = {0.0, 0.0, 0.0};
    double R_NED2Ground[3][3];
    SpeedType<double> vNED[3];
    
    // Get rotation from NED frame to ground frame
    pGround->getGroundAngles(posNED[0].val, posNED[1].val, euler_NED2Ground);
    util.setupRotation(*R_NED2Ground, euler_NED2Ground);
    
    // Rotate Body -> NED, NED -> Ground
    pRotate->bodyToNED(vNED, vBody);
    util.mmult(vGround, *R_NED2Ground, vNED, 3, 3);
}

void ContactPoint::sumExternalForces(void)
{
    if (testGround)
    {
        util.setArray(bodyForceExt, testForceGround, 3);
    }
    else
    {
        util.setArray(bodyForceExt, zero_init, 3);
        
        for(iForceModel curForceModel = pMap->getFirstModel(forceModel); curForceModel != pMap->getModelEnd(forceModel); curForceModel++)
        {
            std::string curModelName = pMap->getModelName(curForceModel);
            
            if( curModelName.compare("DynamicsModel") != 0 && curModelName.compare("GroundModel") != 0 )
            {
                if (debugFlag) { util.print(pMap->getForceModel(curForceModel)->getForce(), 3, curModelName); }
                util.vAdd(bodyForceExt, bodyForceExt, pMap->getForceModel(curForceModel)->getForce(), 3);
            }
        }
    }
    bodyToGround(LLForceExt, bodyForceExt);
    //pRotate->bodyToLL(LLForceExt, bodyForceExt);
}

double ContactPoint::determineFriction(double maxFriction, double externalForce, double velocity, double muDynamic)
{
    double friction = 0;
    int sgn = 1;
    
    // Friction magnitude
    if ( fabs(velocity) < zeroTolerance ) // Use static friction
    {
        // Friction = External Force until maximum friction reached
        if (fabs(externalForce) <= fabs(maxFriction)) { friction = fabs(externalForce); }
        else { friction = std::abs(maxFriction); }
        
        // Direction opposite force
        if (externalForce > 0) { sgn = -1; }
        else { sgn = 1; }
    }
    
    else // Use dynamic friction
    {
        friction = fabs( muDynamic*LLForce[2] );
        
        // Direction opposite velocity
        if (velocity > 0) { sgn = -1; }
        else { sgn = 1; }
    }
    
    // Apply direction
    friction *= sgn;
    
    return friction;
}

// **********************************************************************
// Ground Model Base
// **********************************************************************
GroundModelBase::GroundModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pMap    = pMapInit;
    
    debugFlag = debugFlagIn;
    nContactPoints = 0;
    util.setArray(LLForce, zero_init, 3);
    util.setArray(groundEuler, zero_init, 3);
    
    //pMap->addLogVar("gbxF ", &bodyForce[0], printSavePlot, 3);
    //pMap->addLogVar("gbyF ", &bodyForce[1], printSavePlot, 3);
    //pMap->addLogVar("gbzF ", &bodyForce[2], printSavePlot, 3);
    
    //pMap->addLogVar("gLLxF", &LLForce[0], savePlot, 3);
    //pMap->addLogVar("gLLyF", &LLForce[1], savePlot, 2);
    //pMap->addLogVar("gLLzF", &LLForce[2], savePlot, 3);
    
    //pMap->addLogVar("gbxM ", &bodyMoment[0], savePlot, 2);
    //pMap->addLogVar("gbyM ", &bodyMoment[1], savePlot, 2);
    //pMap->addLogVar("gbzM ", &bodyMoment[2], savePlot, 2);
    
    pMap->addLogVar("Ground Roll", &groundEuler[0], savePlot, 2);
    pMap->addLogVar("Ground Pitch", &groundEuler[1], savePlot, 2);
    pMap->addLogVar("Ground Elevation ", &groundHeight, savePlot, 2);
    pMap->addLogVar("onGround ", &onGroundPrint, savePlot, 2);
    
    for (int i=0; i<maxContactPoints; i++)
    {
        contactPoints[i] = NULL;
    }
    
    std::fill_n(onGround, maxContactPoints, false);
    onGroundPrint = (double) isOnGround();
}

GroundModelBase::~GroundModelBase()
{
    for (int i = 0; i < maxContactPoints; i++)
    {
        if (contactPoints[i] != NULL)
        {
            delete contactPoints[i];
        }
    }
}

void GroundModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    pTime   = (Time*)          pMap->getModel("Time");
    
    nContactPoints = ContactPoint::nContactPoints;
    
    // Initialize contact points
    for (int i = 0; i < nContactPoints; i++)
    {
        contactPoints[i]->initialize();
    }

    setLoadPercentage();
}

void GroundModelBase::setLoadPercentage(void)
{
    // Set percent load for each contact point
    // TODO - use least squares for more / less than 3 contact points
    double A[3][nContactPoints];
    double b[nContactPoints];
    for (int i = 0; i < nContactPoints; i++)
    {
        // Determine how much of a load each contact point carries
        // Setup      A            x    =     b
        //      [ x1  x2  x3 ]  [ pN1 ]     [ 0 ]
        //      [ y1  y2  y3 ]  [ pN2 ] =   [ 0 ]
        //      [ 1   1   1  ]  [ pN3 ]     [ 1 ]
        // Where,
        // xn - x position of nth contact point
        // yn - y position of nth contact point
        // pNn - percent load of nth contact point
    
        double pos[3];
        
        util.setArray(pos, contactPoints[i]->getPos(), 3);
        
        for(int Arow = 0; Arow < nContactPoints-1; Arow++)
        {
            A[Arow][i] = pos[Arow];
        }
        
        A[nContactPoints-1][i] = 1;
        b[i] = 0;
    }
    
    b[nContactPoints-1] = 1;
    
    // TODO - replace if n == 3 with least squares for any number of contact points
    double percN[nContactPoints];
    if (nContactPoints == 3)
    {
        util.LUdecomp(percN, *A, b, nContactPoints);
    }
    else
    {
        for (int i=0; i<nContactPoints; i++) { percN[i] = 1.0/nContactPoints; }
    }
    
    if (debugFlag) util.print(percN,nContactPoints,"Percent Loads:");
    
    for(int i = 0; i < nContactPoints; i ++)
    {
        contactPoints[i]->setPercentLoad( percN[i] );
    }
}

bool GroundModelBase::update(void)
{
    bool continueSim = true;
    
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
    
    for (int i = 0; i < nContactPoints; i++)
    {
        // Update contact points
        contactPoints[i]->update();

        // Determine if on ground
        onGround[i] = contactPoints[i]->isOnGround();
        
        // Sum forces
        util.vAdd(bodyForce, bodyForce, contactPoints[i]->getForce(), 3);
    
        // Sum moments
        util.vAdd(bodyMoment, bodyMoment, contactPoints[i]->getMoment(), 3);
    }
        
    pRotate->bodyToLL(LLForce, bodyForce);
    
    if (debugFlag) util.print(bodyForce, 3, "Ground Total Body Force:");
    if (debugFlag) util.print(bodyMoment, 3, "Ground Total Body Moment:");
    
    AngleType<double> eulerAngles[3];
    util.setUnitClassArray(eulerAngles, pDyn->getEulerAngles(), radians, 3);
    if (debugFlag) util.print(eulerAngles, degrees, 3, "Euler Angles:");

    // Determine if on ground
    static bool groundPrint = false;
    if( isOnGround() )
    {
        if (!groundPrint) { std::cout << "On Ground." << std::endl; groundPrint = true; }
        
        // Determine if in unrecoverable position
        if ( eulerAngles[0].deg() > maxGroundRoll || eulerAngles[0].deg() < minGroundRoll ||
            eulerAngles[1].deg() > maxGroundPitch || eulerAngles[1].deg() < minGroundPitch )
        {
            printf("Ground Model at %2.2fs: angle out of range: roll = %2.2f, pitch = %2.2f\n", pTime->getSimTime(), eulerAngles[0].deg(), eulerAngles[1].deg() );
            continueSim = false;
        }
    }
    else { groundPrint = false; }
    
    onGroundPrint = (double) isOnGround();
    
    groundHeight = getGroundElevation(pDyn->getPosNED()[0], pDyn->getPosNED()[1]);
    getGroundAngles(pDyn->getPosNED()[0], pDyn->getPosNED()[1], groundEuler);
    util.vgain(groundEuler, 1.0/util.deg2rad, 3);
    
    return continueSim;
}

// Get elevation above MSL in meters
double GroundModelBase::getGroundElevation(double north, double east)
{
    //NORTH_ELEVATION_POINTS[N_NORTH_GRID_POINTS];
    //EAST_ELEVATION_POINTS[N_EAST_GRID_POINTS];
    //GRID_ELEVATION[N_NORTH_GRID_POINTS][N_EAST_GRID_POINTS];
    
    // Interpolate east for north low and north high
    double north_elevation[2];
    double north_grid[2];
    double ground_evation;
    int    i_north[2] = {0, N_NORTH_GRID_POINTS};
    int    i = 0;
    
    for (i = 0; i < N_NORTH_GRID_POINTS; i++)
    {
        if (north <= NORTH_ELEVATION_POINTS[i]) { break; }
    }
 
    i_north[0] = util.max(i_north[0], i-1);
    i_north[1] = util.min(i_north[1], i);
    north_grid[0] = NORTH_ELEVATION_POINTS[i_north[0]];
    north_grid[1] = NORTH_ELEVATION_POINTS[i_north[1]];

    north_elevation[0] = util.interpolate(EAST_ELEVATION_POINTS, &GRID_ELEVATION[i_north[0]][0], east, N_EAST_GRID_POINTS);
    north_elevation[1] = util.interpolate(EAST_ELEVATION_POINTS, &GRID_ELEVATION[i_north[1]][0], east, N_EAST_GRID_POINTS);
    ground_evation = util.interpolate(north_grid, north_elevation, north, 2);
    
    return ground_evation;
}

 void GroundModelBase::getGroundAngles(double north, double east, double* ground_euler)
{
    double dx = 0.1;
    double dx_BLL[3] = {dx, 0.0, 0.0};
    double dx_NED[3];
    double euler[3] = {0.0, 0.0, 0.0};
    double R_NED2BLL[3][3];
    double R_BLL2NED[3][3];
    double dh;
    double heading = ground_euler[2];
    for (int i = 0; i < 2; i++)
    {
        if      (i == 0) { euler[2] = heading - M_PI/2.0; }
        else if (i == 1) { euler[2] = heading; }
        
        util.setupRotation(*R_NED2BLL, euler);
        util.mtran(*R_BLL2NED, *R_NED2BLL, 3, 3);
        util.mmult(dx_NED, *R_BLL2NED, dx_BLL, 3, 3);
    
        dh = getGroundElevation(north+dx_NED[0], east+dx_NED[1]) - getGroundElevation(north, east);
    
        ground_euler[i] = atan(dh/dx);
    }
}

// **********************************************************************
// RC Plance Ground Model
// **********************************************************************
RCGroundModel::RCGroundModel(ModelMap *pMapInit, bool debugFlagIn) : GroundModelBase(pMapInit, debugFlagIn)
{

    pMap    = pMapInit;
    debugFlag = debugFlagIn;
    
    // Create contact points
    // Left landing gear
    contactPoints[0] = new ContactPoint(pMapInit,
                                        pos1L[0], pos1L[1], pos1L[2],
                                        k1L, zeta1L,
                                        mux1L_static, muy1L_static,
                                        mux1L_dyn, muy1L_dyn,
                                        debugFlagIn);
    
    // Right landing gear
    contactPoints[1] = new ContactPoint(pMapInit,
                                        pos1R[0], pos1R[1], pos1R[2],
                                        k1R, zeta1R,
                                        mux1R_static, muy1R_static,
                                        mux1R_dyn, muy1R_dyn,
                                        debugFlagIn);
    
    // Rear landing gear
    contactPoints[2] = new ContactPoint(pMapInit,
                                        pos2[0], pos2[1], pos2[2],
                                        k2, zeta2,
                                        mux2_static, muy2_static,
                                        mux2_dyn, muy2_dyn,
                                        debugFlagIn);
}

// **********************************************************************
// Quadcopter Ground Model
// **********************************************************************
QuadcopterGroundModel::QuadcopterGroundModel(ModelMap *pMapInit, bool debugFlagIn) : GroundModelBase(pMapInit, debugFlagIn)
{
    
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    // Create contact points
    // Front left landing gear
    contactPoints[0] = new ContactPoint(pMapInit,
                                        pos1[0], pos1[1], pos1[2],
                                        k1, zeta1,
                                        mux1_static, muy1_static,
                                        mux1_dyn, muy1_dyn,
                                        debugFlagIn);
    
    // Front right landing gear
    contactPoints[1] = new ContactPoint(pMapInit,
                                        pos2[0], pos2[1], pos2[2],
                                        k2, zeta2,
                                        mux2_static, muy2_static,
                                        mux2_dyn, muy2_dyn,
                                        debugFlagIn);
    
    // Rear right landing gear
    contactPoints[2] = new ContactPoint(pMapInit,
                                        pos3[0], pos3[1], pos3[2],
                                        k3, zeta3,
                                        mux3_static, muy3_static,
                                        mux3_dyn, muy3_dyn,
                                        debugFlagIn);
    
    // Rear left landing gear
    contactPoints[3] = new ContactPoint(pMapInit,
                                        pos4[0], pos4[1], pos4[2],
                                        k4, zeta4,
                                        mux4_static, muy4_static,
                                        mux4_dyn, muy4_dyn,
                                        debugFlagIn);
}
