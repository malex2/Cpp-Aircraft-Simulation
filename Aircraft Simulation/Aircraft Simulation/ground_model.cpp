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

int ContactPoint::iContactPoint  = 0;
int ContactPoint::nContactPoints = 0;

ContactPoint::ContactPoint(ModelMap *pMapInit, float x, float y, float z, float kIn, float zetaIn, float muxStaticIn, float muyStaticIn, float muxDynamicIn, float muyDynamicIn, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    
    pMap = pMapInit;
    debugFlag = debugFlagIn;
    
    // position relative to CG
    float posRelCG_in[3];
    posRelCG_in[0] = x;
    posRelCG_in[1] = y;
    posRelCG_in[2] = z;
    util.setUnitClassArray(posRelCG, posRelCG_in, meters, 3);
    
    // Velocity relative to CG
    util.setUnitClassArray(velRelCG, zero_init, metersPerSecond, 3);
    
    // Height above ground
    altGround.convertUnit(meters);
    
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
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    
    percLoad = 1;
    
    // Damping
    float m = pDyn->getMass();
    b  = 2*zeta*sqrt(k*m);
}

bool ContactPoint::update(void)
{
    iContactPoint++;
    
    SpeedType<float> velRel[3];
    SpeedType<float> velBody[3];
    
    // Update position relative to ground
    pRotate->bodyToLL(posLL, posRelCG);
    
    altGround.val = pDyn->gethGround().m() - posLL[2].m();
    
    // Update Local Level velocity
    util.setUnitClassArray(velRel, zero_init, metersPerSecond, 3);
    util.crossProduct(velRel, pDyn->getBodyRates(), posRelCG);
    
    util.vAdd(velBody, pDyn->getVelBody(), velRel, 3);
    
    pRotate->bodyToLL(velLL, velBody);
    
    if (altGround.val <= zeroTolerance ) { onGround = true; }
    else { onGround = false; }
    
    if (onGround)
    {
        float m = pDyn->getMass();
        b  = 2*zeta*sqrt(k*m);
        
        // Update normal force (Local Level force in Z direction)
        LLForce[2] = k*altGround.val - b*velLL[2].val;
        
        // Normal force always up (negative Local Level z direction) or zero
        if (LLForce[2] > 0) { LLForce[2] = 0; }
        
        // Get aerodynamic, atmospheric, and propulsion forces in local level frame
        sumExternalForces();
        
        // Update friction
        float FxMax = muxStatic*LLForce[2];
        LLForce[0] = determineFriction(FxMax, LLForceExt[0]*percLoad, velLL[0].val, muxStatic);
        
        float FyMax = muyStatic*LLForce[2]*percLoad;
        LLForce[1] = determineFriction(FyMax, LLForceExt[1]*percLoad, velLL[1].val, muyDynamic);
    }
    else
    {
        util.setArray(LLForce, zero_init, 3);
    }
    
    pRotate->LLToBody(bodyForce, LLForce);
    
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
                util.vAdd(bodyForceExt, bodyForceExt, pMap->getForceModel(curForceModel)->getForce(), 3);
            }
        }
    }
    
    pRotate->bodyToLL(LLForceExt, bodyForceExt);
}

float ContactPoint::determineFriction(float maxFriction, float externalForce, float velocity, float muDynamic)
{
    float friction = 0;
    int sgn = 1;
    
    // Friction magnitude
    if ( fabs(velocity) < zeroTolerance ) // Use static friction
    {
        // Friction = External Force until maximum friction reached
        if (fabsf(externalForce) <= fabsf(maxFriction)) { friction = fabsf(externalForce); }
        else { friction = fabsf(maxFriction); }
        
        // Direction opposite force
        if (externalForce > 0) { sgn = -1; }
        else { sgn = 1; }
    }
    
    else // Use dynamic friction
    {
        friction = fabsf( muDynamic*LLForce[2] );
        
        // Direction opposite velocity
        if (velocity > 0) { sgn = -1; }
        else { sgn = 1; }
    }
    
    // Apply direction
    friction *= sgn;
    
    return friction;
}

GroundModel::GroundModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pMap    = pMapInit;
    
    //pMap->addLogVar("gbxF ", &bodyForce[0], printSavePlot, 3);
    //pMap->addLogVar("gbyF ", &bodyForce[1], printSavePlot, 3);
    //pMap->addLogVar("gbzF ", &bodyForce[2], printSavePlot, 3);
    
    pMap->addLogVar("gLLxF", &LLForce[0], savePlot, 3);
    pMap->addLogVar("gLLyF", &LLForce[1], savePlot, 2);
    pMap->addLogVar("gLLzF", &LLForce[2], savePlot, 3);
    
    pMap->addLogVar("gbxM ", &bodyMoment[0], savePlot, 2);
    pMap->addLogVar("gbyM ", &bodyMoment[1], savePlot, 2);
    pMap->addLogVar("gbzM ", &bodyMoment[2], savePlot, 2);
    
    util.setArray(LLForce, zero_init, 3);
    
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
    
    std::fill_n(onGround, nContactPoints, false);
    
    debugFlag = debugFlagIn;
}

GroundModel::~GroundModel()
{
    for (int i = 0; i < nContactPoints; i++)
    {
        delete contactPoints[i];
    }
}

void GroundModel::initialize(void)
{
    pDyn    = (DynamicsModel*) pMap->getModel("DynamicsModel");
    pRotate = (RotateFrame*)   pMap->getModel("RotateFrame");
    pTime   = (Time*)          pMap->getModel("Time");
    
    // Initialize contact points
    for (int i = 0; i < nContactPoints; i++)
    {
        contactPoints[i]->initialize();
    }

    setLoadPercentage();
}

void GroundModel::setLoadPercentage(void)
{
    // Set percent load for each contact point
    // TODO - use least squares for more / less than 3 contact points
    float A[3][nContactPoints];
    float b[nContactPoints];
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
    
        float pos[3];
        
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
    if (nContactPoints == 3)
    {
        float percN[nContactPoints];
        util.LUdecomp(percN, *A, b, nContactPoints);
        
        for(int i = 0; i < nContactPoints; i ++)
        {
            contactPoints[i]->setPercentLoad( percN[i] );
        }
    }
}

bool GroundModel::update(void)
{
    bool continueSim = true;
    
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
    
    for (int i = 0; i < nContactPoints; i++)
    {
        if (debugFlag) util.print(&i, 1, "contact point:");
        
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
    
    // Determine if on ground
    if( util.any(onGround, nContactPoints) )
    {
        AngleType<float> *eulerAngles = pDyn->getEulerAngles();
        
        // Determine if in unrecoverable position
        if ( eulerAngles[0].deg() > maxGroundRoll || eulerAngles[0].deg() < minGroundRoll ||
            eulerAngles[1].deg() > maxGroundPitch || eulerAngles[1].deg() < minGroundPitch )
        {
            printf("Ground Model at %2.2fs: angle out of range: roll = %2.2f, pitch = %2.2f\n", pTime->getSimTime(), eulerAngles[0].deg(), eulerAngles[1].deg() );
            continueSim = false;
        }
    }
    
    return continueSim;
}
