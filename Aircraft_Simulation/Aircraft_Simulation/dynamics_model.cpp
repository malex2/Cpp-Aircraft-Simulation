//
//  DynamicsModel.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 7/22/19.
//  Copyright © 2019 Alexander McLean. All rights reserved.
//

#include "initial_conditions.hpp"
#include "model_mapping.hpp"
#include "rotate_frame.hpp"
#include "atmosphere_model.hpp"
#include "time.hpp"

#include "dynamics_model.hpp"

DynamicsModel::DynamicsModel(ModelMap *pMapInit, bool debugFlagIn)
{
    pRotate = NULL;
    pAtmo   = NULL;
    pTime   = NULL;
    
    pMap = pMapInit;
    
    pMap->addLogVar("N (m)", &posRelNED[0].val, savePlot, 2);
    pMap->addLogVar("E (m)", &posRelNED[1].val, savePlot, 2);
    //pMap->addLogVar("Alt  ", &posBodyPrint[2], printSavePlot, 3);
    pMap->addLogVar("gndAlt", &hGroundft, printSavePlot, 3);
    
    pMap->addLogVar("speed", &gndVel.val, printSavePlot, 3);
    pMap->addLogVar("VN  ", &velNED[0].val, savePlot, 2);
    pMap->addLogVar("VE  ", &velNED[1].val, savePlot, 2);
    pMap->addLogVar("VD  ", &velNED[2].val, savePlot, 2);
    
    //pMap->addLogVar("Roll Rate", &eulerRatesDeg[0].val, savePlot, 2);
    //pMap->addLogVar("Pitch Rate", &eulerRatesDeg[1].val, savePlot, 2);
    //pMap->addLogVar("Yaw Rate", &eulerRatesDeg[2].val, savePlot, 2);
    
    //pMap->addLogVar("pdot", &bodyAngularAcc[0], printSavePlot, 3);
    //pMap->addLogVar("qdot", &bodyAngularAcc[1], printSavePlot, 3);
    //pMap->addLogVar("rdot", &bodyAngularAcc[2], printSavePlot, 3);
    
    //pMap->addLogVar("p", &bodyRatesDeg[0].val, savePlot, 2);
    //pMap->addLogVar("q", &bodyRatesDeg[1].val, printSavePlot, 3);
    //pMap->addLogVar("r", &bodyRatesDeg[2].val, savePlot, 2);
    
    pMap->addLogVar("Roll ", &eulerAnglesDeg[0].val, printSavePlot, 3);
    pMap->addLogVar("Pitch", &eulerAnglesDeg[1].val, printSavePlot, 3);
    pMap->addLogVar("Yaw  ", &eulerAnglesDeg[2].val, savePlot, 2);
    
    //pMap->addLogVar("SumXLL", &LLForce[0], printSavePlot, 3);
    //pMap->addLogVar("SumYLL", &LLForce[1], savePlot, 2);
    //pMap->addLogVar("SumZLL", &LLForce[2], printSavePlot, 3);
 
    //pMap->addLogVar("Body Accel X", &accBody[0], printSavePlot, 3);
    //pMap->addLogVar("Body Accel Y", &accBody[1], savePlot, 2);
    //pMap->addLogVar("Body Accel Z", &accBody[2], printSavePlot, 3);
    
    //pMap->addLogVar("SumMX", &bodyMoment[0], savePlot, 2);
    //pMap->addLogVar("SumMY", &bodyMoment[1], savePlot, 2);
    //pMap->addLogVar("SumMZ", &bodyMoment[2], savePlot, 2);
    
    // Initialize Angles
    util.setUnitClassArray(eulerAngles, eulerAngles_init, degrees, 3);
    util.setUnitClassUnit(eulerAngles, radians, 3);
    
    util.eulerToQuaternion(q_B_NED, eulerAngles);
    util.setArray(q_B_NED_dot, quaternion_init2, 3);
    
    // Extract yaw and copmute q_B_LL
    double temp1[3], tempQuat[4];
    util.setArray(temp1, zero_init, 3);
    temp1[2] = -eulerAngles[2].deg(); // -yaw
    util.eulerToQuaternion(tempQuat, temp1); // -yaw in quaternion
    util.quaternionProduct(q_B_LL, tempQuat, q_B_NED);
    
    // Initialize Position
    util.setUnitClassArray(posRelNED, zero_init, meters, 3);
    util.setArray(posBodyPrint, posBody_init, 3);
    
    posBody[0] = posBodyPrint[0] * util.deg2rad;
    posBody[1] = posBodyPrint[1] * util.deg2rad;
    posBody[2] = posBodyPrint[2] * util.ft2m;
    
    elevation.convertUnit(meters);
    elevation.val = elevation_init;
    
    hGround.convertUnit(meters);
    hGround.val = posBody[2] - elevation.m();
    hGroundft = hGround.ft();
    
    hCenter.convertUnit(meters);
    hCenter.val = posBody[2] + Rearth;
    
    // Initialize Velocity
    util.setUnitClassArray(velNED, velNED_init, metersPerSecond, 3);
    util.setUnitClassArray(velBody, zero_init, metersPerSecond, 3);
    util.setUnitClassArray(velBodyRelWind, zero_init, metersPerSecond, 3);
    util.setUnitClassArray(velNEDRelWind, zero_init, metersPerSecond, 3);
    
    gndVel.convertUnit(metersPerSecond);
    gndVel.val = util.mag(velNED, 3);
    
    // Initialize Angle Rates
    util.setUnitClassArray(eulerRates, eulerRates_init, degreesPerSecond, 3);
    util.setUnitClassUnit(eulerRates, radiansPerSecond, 3);
    
    util.setUnitClassArray(bodyRates, zero_init, degreesPerSecond, 3);
    util.setUnitClassUnit(bodyRates, radiansPerSecond, 3);
    
    // Initialize Accelerations
    util.setArray(accBody, zero_init, 3);
    util.setArray(bodyAngularAcc, zero_init, 3);
    
    // Initialize Forces
    util.setArray(bodyForce, zero_init, 3);
    util.setArray(bodyMoment, zero_init, 3);
    
    // Initialize Mass Properties
    util.setMatrix(*inertia, *inertia_init, 3, 3);
    mass = mass_init;
    
    dt = dynamicsInterval_init;
    
    debugFlag = debugFlagIn;
    
    if (debugFlag)
    {
        printf("Dynamics Model Constructor:\n");
        util.print(posRelNED, 3, "posRelNED:");
        util.print(velNED, 3, "velNED:");
        util.print(eulerAngles, degrees, 3, "eulerAngles:");
        util.print(q_B_NED, 4, "q_B_NED:");
        util.print(eulerRates, degreesPerSecond, 3, "eulerRates:");
        util.print(*inertia, 3, 3, "inertia:");
        util.print(&mass, 1, "mass:");
    }
}

void DynamicsModel::initialize(void)
{
    pRotate = (RotateFrame*) pMap->getModel("RotateFrame");
    pAtmo = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pTime   = (Time*) pMap->getModel("Time");
    
    // Body frame dynamics
    pRotate->NEDToBody(velBody, velNED);
    pRotate->eulerRateToBodyRate(bodyRates, eulerRates);
    
    // Wind relative speed
    util.vSubtract(velBodyRelWind, velBody, pAtmo->getVelWindBody(), 3);
    util.vSubtract(velNEDRelWind, velNED, pAtmo->getVelWindNED(), 3);
    
    if (debugFlag)
    {
        printf("Dynamics Model Init:\n");
        util.print(velBody,3,"velBody:");
        util.print(eulerRates, degreesPerSecond, 3, "eulerRates:");
        util.print(bodyRates, degreesPerSecond, 3, "bodyRates:");
    }
}

bool DynamicsModel::update(void)
{
    if (debugFlag) { printf("Dynamics Model Update:\n"); }
    // Setup print variables
    util.setUnitClassUnit(eulerAnglesDeg, radians, 3);
    util.setUnitClassUnit(eulerCheckDeg, radians, 3);
    util.setUnitClassUnit(bodyRatesDeg, radiansPerSecond, 3);
    util.setUnitClassUnit(eulerRatesDeg, radiansPerSecond, 3);
    
    // Update forces
    if (testDynamics)
    {
        util.setArray(bodyForce, testForceDynamics, 3);
        util.setArray(bodyMoment, testMomentDynamics, 3);
    }
    else
    {
        util.setArray(bodyForce, zero_init, 3);
        util.setArray(bodyMoment, zero_init, 3);
        
        for(iForceModel curForceModel = pMap->getFirstModel(forceModel); curForceModel != pMap->getModelEnd(forceModel); curForceModel++)
        {
            std::string curModelName = pMap->getModelName(curForceModel);
            if( curModelName.compare("DynamicsModel") != 0 )
            {
                util.vAdd(bodyForce, bodyForce, pMap->getForceModel(curForceModel)->getForce(), 3);
                util.vAdd(bodyMoment, bodyMoment, pMap->getForceModel(curForceModel)->getMoment(), 3);
                if (debugFlag) { util.print(pMap->getForceModel(curForceModel)->getForce(), 3, curModelName); }
                if (debugFlag) { util.print(pMap->getForceModel(curForceModel)->getMoment(), 3, curModelName); }
            }
        }
    }
    
    pRotate->bodyToLL(LLForce, bodyForce);
    
    // Update body acceleration
    double temp1[3];
    double temp2[3];
    double tempQuat[4];
    AngleType<double> eulerCheck[3];
    util.setUnitClassUnit(eulerCheck, radians, 3);
    
    // accBody = (F - bodyRates x m*velBody) / mass
    util.setArray(temp1, velBody, 3);
    util.vgain(temp1, mass, 3);
    util.crossProduct(temp2, bodyRates, temp1);
    util.vSubtract(accBody, bodyForce, temp2, 3);
    util.vgain(accBody, 1/mass, 3);
    
    // bodyAngularAcc = I^-1 * (M - bodyRates x I*bodyRates)
    util.mmult(temp1, *inertia, bodyRates, 3, 3);
    util.crossProduct(temp2, bodyRates, temp1);
    util.vSubtract(temp2, bodyMoment, temp2, 3);
    util.LUdecomp(bodyAngularAcc, *inertia, temp2, 3);
    
    // Integrate body acceleration
    util.setArray(temp1, accBody, 3);
    util.vgain(temp1, dt, 3);
    util.vAdd(velBody, velBody, temp1, 3);
    
    util.setArray(temp2, bodyAngularAcc, 3);
    util.vgain(temp2, dt, 3);
    util.vAdd(bodyRates, bodyRates, temp2, 3);
    
    // Euler rates (for reference)
    pRotate->bodyRateToEulerRate(eulerRates, bodyRates);
    util.setArray(temp2, eulerRates, 3);
    util.vgain(temp2, dt, 3);
    util.vAdd(eulerCheck, eulerAngles, temp2, 3);
    /*
    // Way 2: q = q*dq, dq = quaterion(omega*dt, bodyRates/omega)
    double angle;
    double bodyRatesUnit[3];
    static double q_B_NED2[4] = {1,0,0,0};
    static bool set2 = false;
    double dQ[4] = {1,0,0,0};
    double eulerAngles2[3] = {0,0,0};
    double eulerError2[3];
    if (!set2) { util.setArray(q_B_NED2, q_B_NED, 4); set2 = true; }
    
    angle = util.mag(bodyRates, 3) / util.deg2rad * dt;
    util.setArray(bodyRatesUnit, bodyRates, 3);
    util.unitVector(bodyRatesUnit, 3);
    util.initQuaternion(dQ, angle, bodyRatesUnit);
    util.quaternionProduct(q_B_NED2, q_B_NED2, dQ);
    util.unitVector(q_B_NED2,4);
    util.quaternionToEuler(eulerAngles2, q_B_NED2);
    */
    // Update body orientation  (qdot = 0.5 * q * [0 wx wy wz])
    util.vecToQuat(tempQuat, bodyRates);
    util.quaternionProduct(q_B_NED_dot, q_B_NED, tempQuat);
    util.setArray(tempQuat, q_B_NED_dot, 4);
    util.vgain(tempQuat, 0.5, 4);
    util.vgain(tempQuat, dt, 4);
    util.vAdd(q_B_NED, q_B_NED, tempQuat, 4);
    util.unitVector(q_B_NED, 4);
    
    util.quaternionToEuler(eulerAngles, q_B_NED);
    /*for (int i=0; i<3; i++) { eulerError2[i] = eulerAngles[i].deg() - eulerAngles2[i]; }
    
    // Way 3:
    static int counter = 0;
    int countRate = 20;
    static double q_B_NED3[4] = {1,0,0,0};
    static bool set3 = false;
    static double dAngles[3] = {0,0,0};
    double dAnglesUnit[3];
    if (!set3) { util.setArray(q_B_NED3, q_B_NED, 4); set3 = true; }
    for (int i=0; i<3; i++) { dAngles[i] += bodyRates[i].dps()*dt; }
    
    if (counter % countRate == 0)
    {
        double angle;
        double dQ[4] = {1,0,0,0};
        double eulerAngles3[3] = {0,0,0};
        double eulerError3[3] = {0,0,0};
        
        angle = util.mag(dAngles, 3);
        util.setArray(dAnglesUnit, dAngles, 3);
        util.unitVector(dAnglesUnit, 3);
        util.initQuaternion(dQ, angle, dAnglesUnit);
        util.quaternionProduct(q_B_NED3, q_B_NED3, dQ);
        util.unitVector(q_B_NED3, 4);
        util.quaternionToEuler(eulerAngles3, q_B_NED3);
        
        std::cout << "Time: " << pTime->getSimTime() << " " << counter << std::endl;
        util.print(q_B_NED, 4, "q_B_NED:");
        util.print(q_B_NED3, 4, "q_B_NED3:");
        
        util.print(dAngles, 3, "dAngles:");
        util.print(eulerAngles, degrees, 3, "eulerAngles:");
        util.print(eulerAngles3, 3, "eulerAngles3:");
        
        for (int i=0; i<3; i++) { dAngles[i] = 0.0; }
        
        for (int i=0; i<3; i++) { eulerError3[i] = eulerAngles[i].deg() - eulerAngles3[i]; }
        util.print(eulerError3, 3, "eulerError3:");
        
        
        util.print(q_B_NED, 4, "q_B_NED:");
        util.print(q_B_NED2, 4, "q_B_NED2:");
        
        util.print(eulerAngles, degrees, 3, "eulerAngles:");
        util.print(eulerAngles2, 3, "eulerAngles2:");
        util.print(eulerError2, 3, "eulerError2:");
    }
    counter++;
    */
    // Extract yaw and copmute q_B_LL
    util.setArray(temp1, zero_init, 3);
    temp1[2] = -eulerAngles[2].deg(); // -yaw
    util.eulerToQuaternion(tempQuat, temp1); // -yaw in quaternion
    util.quaternionProduct(q_B_LL, tempQuat, q_B_NED);
    
    // Update rotations
    pRotate->update();
    
    // NED frame velocity
    pRotate->bodyToNED(velNED, velBody);
    
    // Update other velocities
    gndVel.val = util.mag(velNED, 3);
    util.vSubtract(velBodyRelWind, velBody, pAtmo->getVelWindBody(), 3);
    util.vSubtract(velNEDRelWind, velNED, pAtmo->getVelWindNED(), 3);
    
    // Relative NED movement
    util.setArray(temp1, velNED, 3);
    util.vgain(temp1, dt, 3);
    util.vAdd(posRelNED, posRelNED, temp1, 3);
    
    // Altitude
    dPosBody[2] = -velNED[2].mps(); // Altitude rate in m/s
    posBody[2]  = posBody[2] + dPosBody[2]*dt;
    
    hGround.val = posBody[2] - elevation.m();
    hGroundft   = hGround.ft();
    hCenter.val = Rearth + posBody[2];
    
    // Latitude and Longitude
    dPosBody[0] = velNED[0].mps() / ( hCenter.m() );  // Latitude rate in rad/s
    dPosBody[1] = velNED[1].mps() / ( hCenter.m() );  // Longitude rate in rad/s
    
    posBody[0] = posBody[0] + dPosBody[0]*dt;
    posBody[1] = posBody[1] + dPosBody[1]*dt;
    
    // Store print variables
    posBodyPrint[0] = posBody[0]/util.deg2rad;
    posBodyPrint[1] = posBody[1]/util.deg2rad;
    posBodyPrint[2] = posBody[2]/util.ft2m;
    
    util.setArray(eulerAnglesDeg, eulerAngles, 3);
    util.setArray(eulerCheckDeg, eulerCheck, 3);
    util.setArray(bodyRatesDeg, bodyRates, 3);
    util.setArray(eulerRatesDeg, eulerRates, 3);
    
    util.setUnitClassUnit(eulerAnglesDeg, degrees, 3);
    util.setUnitClassUnit(eulerCheckDeg, degrees, 3);
    util.setUnitClassUnit(bodyRatesDeg, degreesPerSecond, 3);
    util.setUnitClassUnit(eulerRatesDeg, degreesPerSecond, 3);
    
    if (debugFlag)
    {
        util.print(bodyForce, 3, "bodyForce:");
        util.print(accBody, 3, "accBody:");
        util.print(velBody, 3, "velBody:");
        util.print(velNED, 3, "velNED:");
        util.print(posRelNED, 3, "posRelNED:");
        
        util.print(bodyMoment, 3, "bodyMoment:");
        util.print(bodyAngularAcc, 3, "bodyAngularAcc:");
        util.print(bodyRates, degreesPerSecond, 3, "bodyRates:");
        util.print(eulerRates, degreesPerSecond, 3, "eulerRates:");
        util.print(eulerAngles, degrees, 3, "eulerAngles:");
        util.print(eulerCheck, degrees, 3, "eulerCheck:");
    }
    
    if ( eulerAngles[1].deg() > maxPitch || eulerAngles[1].deg() < minPitch )
    {
        printf("Dynamics at %2.2fs: pitch out of range: %2.2f\n", pTime->getSimTime(), eulerAngles[1].deg() );
        return false;
    }
    else { return true; }
}

// Setters
void DynamicsModel::setEulerAngles(AngleType<double>* angles_in)
{
    for (int i = 0; i < 3; i++)
    {
        eulerAngles[i].val = angles_in[i].rad();
    }
    util.eulerToQuaternion(q_B_NED, eulerAngles);
    pRotate->update();
}

void DynamicsModel::setSpeed(SpeedType<double> vel_in)
{
    SpeedType<double> velLL[3]
    {
        SpeedType<double>(0,metersPerSecond),
        SpeedType<double>(0,metersPerSecond),
        SpeedType<double>(0,metersPerSecond)
    };
    gndVel.val = vel_in.mps();
    
    velLL[0].val = gndVel.val;
    
    pRotate->LLToBody(velBody, velLL);
    pRotate->NEDToBody(velNED, velBody);
}
