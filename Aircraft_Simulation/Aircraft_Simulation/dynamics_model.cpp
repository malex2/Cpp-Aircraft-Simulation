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
    
    //pMap->addLogVar("Dyn deltaCount", &deltaCount, savePlot, 2);
    //pMap->addLogVar("Lat", &posLLH_deg[0], savePlot, 2);
    //pMap->addLogVar("Lon", &posLLH_deg[1], savePlot, 2);
    //pMap->addLogVar("Alt", &posLLH[2], printSavePlot, 3);
    
    //pMap->addLogVar("posECEF X", &posECEF[0], savePlot, 2);
    //pMap->addLogVar("posECEF Y", &posECEF[1], savePlot, 2);
    //pMap->addLogVar("posECEF Z", &posECEF[2], savePlot, 2);
    
    pMap->addLogVar("N (m)", &posNED[0], savePlot, 2);
    pMap->addLogVar("E (m)", &posNED[1], savePlot, 2);
    //pMap->addLogVar("D (m)", &posNED[2], savePlot, 2);
    pMap->addLogVar("gndAlt", &hGround, printSavePlot, 3);
    
    //pMap->addLogVar("speed", &velMag, savePlot, 2);
    
    
    //pMap->addLogVar("velECEF X", &velECEF[0], savePlot, 2);
    //pMap->addLogVar("velECEF Y", &velECEF[1], savePlot, 2);
    //pMap->addLogVar("velECEF Z", &velECEF[2], savePlot, 3);
    
    //pMap->addLogVar("VbX  ", &velBody[0], savePlot, 2);
    //pMap->addLogVar("VbY  ", &velBody[1], savePlot, 2);
    //pMap->addLogVar("VbZ  ", &velBody[2], savePlot, 3);
    
    pMap->addLogVar("VN  ", &velNED[0], savePlot, 2);
    pMap->addLogVar("VE  ", &velNED[1], savePlot, 2);
    //pMap->addLogVar("VD  ", &velNED[2], printSavePlot, 3);
    
    //pMap->addLogVar("Roll Rate", &eulerRatesDeg[0].val, savePlot, 2);
    //pMap->addLogVar("Pitch Rate", &eulerRatesDeg[1].val, savePlot, 2);
    //pMap->addLogVar("Yaw Rate", &eulerRatesDeg[2].val, savePlot, 2);
    
    //pMap->addLogVar("pdot", &bodyAngularAcc[0], printSavePlot, 3);
    //pMap->addLogVar("qdot", &bodyAngularAcc[1], savePlot, 2);
    //pMap->addLogVar("rdot", &bodyAngularAcc[2], printSavePlot, 3);
    
    pMap->addLogVar("p", &bodyRatesDeg[0], savePlot, 2);
    pMap->addLogVar("q", &bodyRatesDeg[1], savePlot, 2);
    pMap->addLogVar("r", &bodyRatesDeg[2], savePlot, 2);
    
    pMap->addLogVar("Roll ", &eulerAnglesDeg[0], savePlot, 2);
    pMap->addLogVar("Pitch", &eulerAnglesDeg[1], savePlot, 2);
    pMap->addLogVar("Yaw  ", &eulerAnglesDeg[2], savePlot, 2);
    
    //pMap->addLogVar("q_B_NED[0]", &q_B_NED[0], savePlot, 2);
    //pMap->addLogVar("q_B_NED[1]", &q_B_NED[1], savePlot, 2);
    //pMap->addLogVar("q_B_NED[2]", &q_B_NED[2], savePlot, 2);
    //pMap->addLogVar("q_B_NED[3]", &q_B_NED[3], savePlot, 2);
    
    //pMap->addLogVar("Roll Int", &eulerAngles_integralTheta[0], savePlot, 2);
    //pMap->addLogVar("Pitch Int", &eulerAngles_integralTheta[1], printSavePlot, 2);
    
    //pMap->addLogVar("Roll Int Err", &eulerError[0], savePlot, 2);
    //pMap->addLogVar("Pitch Int Err", &eulerError[1], printSavePlot, 2);
    
    //pMap->addLogVar("ECI Force X", &forceECI[0], savePlot, 2);
    //pMap->addLogVar("ECI Force Y", &forceECI[1], savePlot, 2);
    //pMap->addLogVar("ECI Force Z", &forceECI[2], savePlot, 2);
    
    //pMap->addLogVar("ECI Accel X", &accelECI[0], savePlot, 2);
    //pMap->addLogVar("ECI Accel Y", &accelECI[1], savePlot, 2);
    //pMap->addLogVar("ECI Accel Z", &accelECI[2], savePlot, 2);
    
    //pMap->addLogVar("Body Force X", &bodyForce[0], savePlot, 2);
    //pMap->addLogVar("Body Force Y", &bodyForce[1], savePlot, 2);
    //pMap->addLogVar("Body Force Z", &bodyForce[2], savePlot, 2);
 
    //pMap->addLogVar("Body Accel X", &accelBody[0], savePlot, 2);
    //pMap->addLogVar("Body Accel Y", &accelBody[1], savePlot, 2);
    pMap->addLogVar("Body Accel Z", &accelBody[2], savePlot, 2);
    //pMap->addLogVar("Accel Mag", &accelMag, savePlot, 2);
    
    //pMap->addLogVar("SumMX", &bodyMoment[0], savePlot, 2);
    //pMap->addLogVar("SumMY", &bodyMoment[1], savePlot, 2);
    //pMap->addLogVar("SumMZ", &bodyMoment[2], savePlot, 2);
    
    util.initArray(forceECI, 0.0, 3);
    util.initArray(accelECI, 0.0, 3);
    util.initArray(accelBody, 0.0, 3);
    accelMag = 0.0;
    util.initArray(velECI, 0.0, 3);
    util.initArray(velNED, 0.0, 3);
    util.initArray(velBody, 0.0, 3);
    util.initArray(velLL, 0.0, 3);
    velMag = 0.0;
    
    util.initArray(posECI, 0.0, 3);
    util.initArray(posLLH, 0.0, 3);
    util.initArray(posECEF, 0.0, 3);
    util.initArray(posECEF_init, 0.0, 3);
    util.initArray(d_posECEF, 0.0, 3);
    util.initArray(posNED, 0.0, 3);
    
    hCenter = 0.0;
    hGround = 0.0;
    hMSL    = 0.0;
    groundElevation = elevation_init;
    
    util.initArray(bodyAngularAcc, 0.0, 3);
    util.initArray(bodyRates, 0.0, 3);
    
    util.initArray(q_B_NED, 0.0, 4);
    util.initArray(dq_B_NED, 0.0, 4);
    util.initArray(eulerAngles, 0.0, 3);
    
    util.setMatrix(*inertia, *inertia_init, 3, 3);
    mass = mass_init;
    timestamp = 0.0;
    dt = dynamicsInterval_init;
    
    // print variables
    util.initArray(posLLH_deg, 0.0, 3);
    util.initArray(bodyRatesDeg, 0.0, 3);
    util.initArray(eulerAnglesDeg, 0.0, 3);
   
    debugFlag = debugFlagIn;
    
    if (debugFlag)
    {
        printf("Dynamics Model Constructor:\n");
        util.print(posNED, 3, "posNED:");
        util.print(velNED, 3, "velNED:");
        util.print(eulerAngles, degrees, 3, "eulerAngles:");
        util.print(q_B_NED, 4, "q_B_NED:");
        util.print(bodyRates, 3, "bodyRates:");
        util.print(*inertia, 3, 3, "inertia:");
        util.print(&mass, 1, "mass:");
    }
}

void DynamicsModel::initialize(void)
{
    pRotate = (RotateFrame*) pMap->getModel("RotateFrame");
    pAtmo   = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pTime   = (Time*) pMap->getModel("Time");

    // Initialize Position
    util.setArray(posLLH, posLLH_init, 3);
    posLLH[0] *= util.deg2rad;
    posLLH[1] *= util.deg2rad;
    posLLH[2] *= util.ft2m;
    
    util.LLHtoECEF(posECEF_init, posLLH);
    util.setArray(posECEF, posECEF_init, 3);
    
    pRotate->ECEFToECI(posECI, posECEF);
    
    // Initialize Velocity
    util.setArray(velNED, velNED_init, 3);
    pRotate->NEDToECEF(velECEF, velNED);
    pRotate->ECEFToECI(velECI, velECEF);
    
    // Initialize Attitude
    util.setArray(eulerAngles, eulerAngles_init, 3);
    util.vgain(eulerAngles, util.deg2rad, 3);
    util.eulerToQuaternion(q_B_NED, eulerAngles);
    
    util.setArray(bodyRates, bodyRates_init, 3);
    util.vgain(bodyRates, util.deg2rad, 3);
    
    updateStates();
    
    if (debugFlag)
    {
        printf("Dynamics Model Init:\n");
        util.print(velBody,3,"velBody:");
        util.print(bodyRates, 3, "bodyRates:");
    }
}

bool DynamicsModel::update(void)
{
    updateDt(pTime);
    
    sumForces();
    updatePosition();
    updateAttitude();
    updateStates();
    
    timestamp = time;
    
    if (debugFlag)
    {
        printf("Dynamics Model Update:\n");
        util.print(forceECI ,3, "forceECI:");
        util.print(accelECI, 3, "accelECI:");
        util.print(accelBody, 3, "accelBody:");
        
        util.print(posECEF, 3, "posECEF:");
        util.print(posLLH, 3, "posLLH:");
        util.print(posNED, 3, "posNED:");
        
        util.print(velECEF, 3, "velECEF:");
        util.print(velNED, 3, "velNED:");
        
        util.print(eulerAnglesDeg, 3, "eulerAngles");
        util.print(bodyRatesDeg, 3, "bodyRates:");
    }
    
    return true;
}

void DynamicsModel::sumForces()
{
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
    pRotate->bodyToECI(forceECI, bodyForce);
}

void DynamicsModel::updatePosition()
{
    for (int i = 0; i < 3; i++)
    {
        accelECI[i] = forceECI[i]/mass;
        posECI[i] = posECI[i] + velECI[i]*dt;// + 0.5*accelECI[i]*dt*dt;
        velECI[i] += accelECI[i]*dt;
    }
}

void DynamicsModel::updateAttitude()
{
    //  variables
    double Iw[3];
    double w_cross_Iw[3];
    double M_minus_w_cross_Iw[3];
    double prev_q_B_NED[4];
    double dTheta[3];
    double dThetaMag;
    double dThetaUnit[3] = {0.0, 0.0, 0.0};
    
    // Angular Acceleration
    // bodyAngularAcc = I^-1 * (M - bodyRates x I*bodyRates)
    util.mmult(Iw, *inertia, bodyRates, 3, 3);
    util.crossProduct(w_cross_Iw, bodyRates, Iw);
    util.vSubtract(M_minus_w_cross_Iw, bodyMoment, w_cross_Iw, 3);
    util.LUdecomp(bodyAngularAcc, *inertia, M_minus_w_cross_Iw, 3);
    
    // Angular Rates
    for (int i = 0; i < 3; i++)
    {
        bodyRates[i] += bodyAngularAcc[i]*dt;
        dTheta[i] = bodyRates[i]*dt;
    }
    
    // Update quaternion
    util.setArray(prev_q_B_NED, q_B_NED, 4);
    dThetaMag = util.mag(dTheta, 3);
    util.setArray(dThetaUnit, dTheta, 3);
    util.unitVector(dThetaUnit, 3);
    
    util.initQuaternion(dq_B_NED, dThetaMag, dThetaUnit);
    util.quaternionProduct(q_B_NED, prev_q_B_NED, dq_B_NED);
    util.unitVector(q_B_NED, 4);
}

void DynamicsModel::updateStates()
{
    pRotate->update();
    
    // ECEF Position
    pRotate->ECIToECEF(posECEF, posECI);
    pRotate->ECIToECEF(velECEF, velECI);
    
    // NED Position
    util.vSubtract(d_posECEF, posECEF, posECEF_init, 3);
    pRotate->ECEFToNED(posNED, d_posECEF);
    pRotate->ECEFToNED(velNED, velECEF);
    
    // Lat/Lon/Altitude
    util.ECEFtoLLH(posLLH, posECEF, 0.0);
    
    // Height references
    hCenter = posLLH[2] + EARTHCONSTANTS::RE;
    hGround = posLLH[2] - groundElevation;
    hMSL    = posLLH[2] + 32.0;
    
    // Body states
    pRotate->ECIToBody(velBody, velECI);
    pRotate->ECIToBody(accelBody, accelECI);
    util.quaternionToEuler(eulerAngles, q_B_NED);
    
    // Local Level states
    pRotate->bodyToLL(velLL, velBody);
    
    // Magnitudes
    velMag   = util.mag(velECI, 3);
    accelMag = util.mag(accelECI, 3);
    
    // Print Variables
    posLLH_deg[0] = posLLH[0] / util.deg2rad;
    posLLH_deg[1] = posLLH[1] / util.deg2rad;
    for (int i = 0; i < 3; i++)
    {
        bodyRatesDeg[i] = bodyRates[i] / util.deg2rad;
        eulerAnglesDeg[i] = eulerAngles[i] / util.deg2rad;
    }
}
