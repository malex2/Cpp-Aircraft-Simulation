//
//  imu_model.cpp
//  Aircraft Simulation
//
//  Created by Alexander McLean on 12/28/20.
//  Copyright © 2020 Alexander McLean. All rights reserved.
//

#include "imu_model.hpp"

 #include "initial_conditions.hpp"
 #include "model_mapping.hpp"
 #include "rotate_frame.hpp"
 #include "dynamics_model.hpp"
 #include "atmosphere_model.hpp"
 #include "time.hpp"
 
IMUModelBase::IMUModelBase(ModelMap *pMapInit, bool debugFlagIn)
{
    pDyn    = NULL;
    pAtmo   = NULL;
    pRotate = NULL;
    pTime   = NULL;
    pMap    = pMapInit;
    
    // units to LSB
    LSBdps = 0.0;
    LSBg   = 0.0;
    LSBuT  = 0.0;
    refGravity = 9.81;;
    perfectSensor = false;;
    imuReady = false;
    onlyGravity = false;
    
    // Noise
    util.initArray(noise, 0.0, 3);
    
    util.initArray(gyroBias, 0.0, 3);
    util.initArray(gyroNoiseMax, 0.0, 3);
    util.initArray(gyroError, 0.0, 3);
    
    util.initArray(accBias, 0.0, 3);
    util.initArray(accNoiseMax, 0.0, 3);
    util.initArray(accError, 0.0, 3);

    util.initArray(magBias, 0.0, 3);
    util.initArray(magNoiseMax, 0.0, 3);
    util.initMatrix(*SoftIronMatrix, 0.0, 3, 3);
    util.initArray(magWithSoftIron, 0.0, 3);
    util.initArray(magError, 0.0, 3);
    
    util.initArray(gyroSensor, 0.0, 3);
    util.initArray(accSensor, 0.0, 3);
    util.initArray(magSensor, 0.0, 3);
 
    util.initArray(gyroInUnits, 0.0, 3);
    util.initArray(accInUnits, 0.0, 3);
    util.initArray(magInUnits, 0.0, 3);
    
    util.setUnitClassArray(bodyRates, zero_init, radiansPerSecond, 3);
    util.setUnitClassArray(eulerAngles, zero_init, degrees, 3);
    util.initArray(bodyAcc, 0.0, 3);
    util.initArray(bodyAngularAcc, 0.0, 3);
    gravity          = 9.81;
    magneticStrength = 55.0;
    inclination      = 70.0;
    
    util.initArray(gyroIMU, 0.0, 3);
    
    util.initArray(accTangentBody, 0.0, 3);
    util.initArray(accNormalBody, 0.0, 3);
    util.setUnitClassArray(velNormalBody, zero_init, metersPerSecond, 3);
    util.initArray(accRotationBody, 0.0, 3);
    util.initArray(accTotalBody, 0.0, 3);
    util.initArray(gravityNED, 0.0, 3);
    util.initArray(gravityBody, 0.0, 3);
    util.initArray(gravityIMU, 0.0, 3);
    util.initArray(accIMUnoGravity, 0.0, 3);
    util.initArray(accIMU, 0.0, 3);
    
    util.initArray(magNED, 0.0, 3);
    util.initArray(magBody, 0.0, 3);
    util.initArray(magIMU, 0.0, 3);

    util.setUnitClassArray(sensorFrameEuler, zero_init, degrees, 3);
    sensorFrameEuler[0].val = 180.0;
    sensorFrameEuler[1].val = 0.0;
    sensorFrameEuler[2].val = 0.0;
    
    util.initArray(dTheta, 0.0, 3);
    util.initArray(dVelocity, 0.0, 3);
    sumTime     = 0.0;
    resetPeriod = dynamicsInterval_init;
    
    util.setUnitClassArray(sensorFramePosition, zero_init, meters, 3);
    
    debugFlag = debugFlagIn;
    
    //pMap->addLogVar("IMU gyro x Error (deg/s)", &gyroError[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y Error (deg/s)", &gyroError[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z Error (deg/s)", &gyroError[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x Error (g)", &accError[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y Error (g)", &accError[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z Error (g)", &accError[2], savePlot, 2);
    //pMap->addLogVar("IMU mag x Error (uT)", &magError[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y Error (uT)", &magError[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z Error (uT)", &magError[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gyro x (deg/s)", &gyroInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y (deg/s)", &gyroInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z (deg/s)", &gyroInUnits[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x (g)", &accInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y (g)", &accInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z (g)", &accInUnits[2], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc x (g)", &accTangentBody[0], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc y (g)", &accTangentBody[1], savePlot, 2);
    //pMap->addLogVar("IMU tangential acc z (g)", &accTangentBody[2], savePlot, 2);
    //pMap->addLogVar("IMU normal acc x (g)", &accNormalBody[0], savePlot, 2);
    //pMap->addLogVar("IMU normal acc y (g)", &accNormalBody[1], savePlot, 2);
    //pMap->addLogVar("IMU normal acc z (g)", &accNormalBody[2], savePlot, 2);
    //pMap->addLogVar("dPitch", &dTheta[1], printSavePlot, 3);
    //pMap->addLogVar("dVelocity X", &dVelocity[0], savePlot, 2);
    //pMap->addLogVar("dVelocity Y", &dVelocity[1], savePlot, 2);
    //pMap->addLogVar("dVelocity Z", &dVelocity[2], savePlot, 2);
    //pMap->addLogVar("IntSumTime", &sumTime, printSavePlot, 3);
    
    //pMap->addLogVar("IMU mag x (uT)", &magInUnits[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y (uT)", &magInUnits[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z (uT)", &magInUnits[2], savePlot, 2);
    
    //pMap->addLogVar("IMU gyro x (raw)", &gyroSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU gyro y (raw)", &gyroSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU gyro z (raw)", &gyroSensor[2], savePlot, 2);
    //pMap->addLogVar("IMU acc x (raw)", &accSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU acc y (raw)", &accSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU acc z (raw)", &accSensor[2], savePlot, 2);
    //pMap->addLogVar("IMU mag x (raw)", &magSensor[0], savePlot, 2);
    //pMap->addLogVar("IMU mag y (raw)", &magSensor[1], savePlot, 2);
    //pMap->addLogVar("IMU mag z (raw)", &magSensor[2], savePlot, 2);
}

void IMUModelBase::initialize(void)
{
    pDyn    = (DynamicsModel*)   pMap->getModel("DynamicsModel");
    pAtmo   = (AtmosphereModel*) pMap->getModel("AtmosphereModel");
    pRotate = (RotateFrame*)     pMap->getModel("RotateFrame");
    pTime   = (Time*)            pMap->getModel("Time");
}

bool IMUModelBase::update(void)
{
    util.setUnitClassUnit(bodyRates, radiansPerSecond, 3);
    
    // Update States
    util.setArray(bodyRates  , pDyn->getBodyRates()  , 3);
    util.setArray(eulerAngles, pDyn->getEulerAngles(), 3);
    util.setArray(bodyAcc    , pDyn->getAccBody()    , 3);
    util.setArray(bodyAngularAcc, pDyn->getbodyAngularAcc(), 3);
    gravity        = pAtmo->getGravity();
    refGravity     = gravity; // Check and udpate
    //magneticStrength = pAtmo->getMAgneticFieldStrength();
    //inclination      = pDyn->getInclination();
    
    // Update Units
    util.setUnitClassUnit(bodyRates, degreesPerSecond, 3);
    util.setArray(bodyRatesPrint, bodyRates, 3);
    
    // Update IMU measurements
    gyroscopeModel();
    accelerometerModel();
    magnetometerModel();
    
    imuReady = true;
    
    if (debugFlag)
    {
        // Gyroscope
        util.print(bodyRates, 3, "Body Rates");
        util.print(gyroIMU, 3, "True IMU Gyroscope");
        util.print(gyroError, 3, "Gyroscope Errors");
        util.print(gyroInUnits, 3, "Gyroscope In Units");
        util.print(gyroSensor, 3, "Gyroscope Raw");
        
        // Accelerometer
        util.print(bodyAcc, 3, "Body Acceleration");
        util.print(accTangentBody, 3, "Body Tangent Acceleration");
        util.print(accNormalBody, 3, "Body Normal Acceleration");
        util.print(accTotalBody, 3, "Body Total Acceleration");
        util.print(gravityIMU, 3, "Gravity In IMU Frame");
        util.print(accIMU, 3, "True IMU Accelerometer");
        util.print(accError, 3, "Accelerometer Errors");
        util.print(accInUnits, 3, "Acclerometer In Units");
        util.print(accSensor, 3, "Accelerometer Raw");
        
        // Integrals
        util.print(dTheta, 3, "dTheta");
        util.print(dVelocity, 3, "dVelocity");
        
        // Magnetometer
        util.print(magNED, 3, "NED Magnetic Field");
        util.print(magIMU, 3, "True IMU Magnetometer");
        util.print(magError, 3, "Magnetometer Errors");
        util.print(magWithSoftIron, 3, "Magnetometer With Soft Iron");
        util.print(magInUnits, 3, "Magnetometer In Units");
        util.print(magSensor, 3, "Magnetometer Raw");
    }
    
    return true;
}

void IMUModelBase::gyroscopeModel(void)
{
    pRotate->bodyToImu(gyroIMU, bodyRates);
    
    // Add Error
    util.vAdd(gyroError, gyroBias, noiseModel(gyroNoiseMax), 3);
    
    if (perfectSensor)
    {
        util.setArray(gyroInUnits, gyroIMU, 3);
    }
    else
    {
        util.vAdd(gyroInUnits, gyroIMU, gyroError, 3);
    }
    
    // Raw Body Rate
    util.setArray(gyroSensor, gyroInUnits, 3);
    util.vgain(gyroSensor, LSBdps, 3);
}

void IMUModelBase::accelerometerModel(void)
{
    // A_imu = A_body + cross(angleAcc, r) + angleRate x (angleRate x r);
    // A_imu = A_body +     A_tangent      +         A_normal
    
    // Tangent Acceleration
    util.crossProduct(accTangentBody, bodyAngularAcc, sensorFramePosition);
    
    // Normal Acceleration
    util.crossProduct(velNormalBody, bodyRates, sensorFramePosition);
    util.crossProduct(accNormalBody, bodyRates, velNormalBody);
    
    // Total Body Acceleration
    util.vAdd(accRotationBody, accTangentBody, accNormalBody, 3);
    util.vAdd(accTotalBody, bodyAcc, accRotationBody, 3);
    
    // Gravity
    gravityNED[2] = gravity;
    pRotate->NEDToBody(gravityBody, gravityNED);
    pRotate->bodyToImu(gravityIMU, gravityBody);
    // Gravity relative to free-fall
    util.vgain(gravityIMU, -1.0, 3);
    
    // IMU Acceleration
    // Acceleration in IMU frame
    pRotate->bodyToImu(accIMUnoGravity, accTotalBody);
    
    // Add gravity
    util.vAdd(accIMU, accIMUnoGravity, gravityIMU, 3);
    
    if (onlyGravity) { util.setArray(accIMU, gravityIMU, 3); }
        
    // Acceleration in g's
    util.vgain(accIMU, 1/refGravity, 3);
    
    // Add Error
    util.vAdd(accError, accBias, noiseModel(accNoiseMax), 3);
    
    if (perfectSensor)
    {
        util.setArray(accInUnits, accIMU, 3);
    }
    else
    {
        util.vAdd(accInUnits, accIMU, accError, 3);
    }
    
    // Raw Acceleration
    util.setArray(accSensor, accInUnits, 3);
    util.vgain(accSensor, LSBg, 3);
}

void IMUModelBase::deltaIMU(double dt)
{
    for (int i=0; i<3; i++)
    {
        dTheta[i]    += gyroIMU[i]*dt;
        dVelocity[i] += accIMUnoGravity[i]*dt;
    }
    sumTime += dt;
}

void IMUModelBase::reset()
{
    pDyn->updateIntegralQuaternion(dTheta, sumTime);
    
    util.setArray(dTheta, zero_init, 3);
    util.setArray(dVelocity, zero_init, 3);
    sumTime = 0.0;
}

void IMUModelBase::magnetometerModel(void)
{
    // Earth magnetic field
    magNED[0] = magneticStrength*cos(inclination*util.deg2rad);
    magNED[1] = 0;
    magNED[2] = magneticStrength*sin(inclination*util.deg2rad);
    
    // Rotate Earth magnetic field into sensor frame
    pRotate->NEDToBody(magBody, magNED);
    pRotate->bodyToImu(magIMU, magBody);

    // Add Error
    util.vAdd(magError, magBias, noiseModel(magNoiseMax), 3);
    
    if (perfectSensor)
    {
        util.setArray(magInUnits, magIMU, 3);
    }
    else
    {
        util.mmult(magWithSoftIron, *SoftIronMatrix, magIMU, 3, 1);
        util.vAdd(magInUnits, magWithSoftIron, magError, 3);
    }
    
    // Raw Magnetic Field
    util.setArray(magSensor, magInUnits, 3);
    util.vgain(magSensor, LSBg, 3);
}

double* IMUModelBase::noiseModel(double* maxNoise)
{
    util.initArray(noise, 0.0, 3);
    for (int i=0; i<3; i++)
    {
        noise[i] = noiseModel( *(maxNoise+i) );
    }
    return &noise[0];
}

double IMUModelBase::noiseModel(double maxNoise)
{
    // update random seed
    static int count = 0;
    srand(++count);
    
    int randInt = rand() % 100;
    double randToMax = maxNoise*randInt / 100.0;
    double randMinMax = randToMax*2 - maxNoise;
    return randMinMax;
}

QuadcopterIMUModel::QuadcopterIMUModel(ModelMap *pMapInit, bool debugFlagIn) : IMUModelBase(pMapInit, debugFlagIn)
{
    // LSB
    LSBdps = 32.8;
    LSBg   = 4096;
    LSBuT  = 16;
    
    // Noise
    gyroBias[0] = 0.02;
    gyroBias[1] = -0.08;
    gyroBias[2] = 0.05;
    gyroNoiseMax[0] = 0.23;//0.15;
    gyroNoiseMax[1] = 0.27;//0.35;
    gyroNoiseMax[2] = 0.26;//0.15;
    
    accBias[0] = 0.01;
    accBias[1] = -0.03;
    accBias[2] = 0.08;
    accNoiseMax[0] = 0.02;
    accNoiseMax[1] = 0.02;
    accNoiseMax[2] = 0.02;

    magBias[0] = 1.0;
    magBias[1] = -2.0;
    magBias[2] = -23.0;
    magNoiseMax[0] = 0.5;
    magNoiseMax[1] = 1.0;
    magNoiseMax[2] = 0.5;
    SoftIronMatrix[0][0] = 1.0; SoftIronMatrix[0][1] = 0.0; SoftIronMatrix[0][2] = 0.0;
    SoftIronMatrix[1][0] = 0.0; SoftIronMatrix[1][1] = 1.0; SoftIronMatrix[1][2] = 0.0;
    SoftIronMatrix[2][0] = 0.0; SoftIronMatrix[2][1] = 0.0; SoftIronMatrix[2][2] = 1.0;

    magneticStrength = 55;
    inclination = 70;
    
    resetPeriod = 1.0/50.0;
    
    onlyGravity = false;
}
